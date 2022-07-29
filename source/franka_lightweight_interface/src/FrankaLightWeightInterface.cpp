#include "franka_lightweight_interface/FrankaLightWeightInterface.hpp"

class IncompatibleControlTypeException : public std::runtime_error {
public:
  explicit IncompatibleControlTypeException(const std::string& msg) : runtime_error(msg) {};
};

namespace frankalwi {

static CollisionBehaviour default_collision_behaviour() {
  return {{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}};
}

static Eigen::Array<double, 7, 1> default_damping_gains() {
  Eigen::ArrayXd gains = Eigen::ArrayXd(7);
  gains << 25.0, 25.0, 25.0, 25.0, 15.0, 15.0, 5.0;
  return gains;
}

FrankaLightWeightInterface::FrankaLightWeightInterface(
    std::string robot_ip, std::string state_uri, std::string command_uri, std::string prefix
) :
    prefix_(std::move(prefix)),
    robot_ip_(std::move(robot_ip)),
    connected_(false),
    shutdown_(false),
    state_uri_(std::move(state_uri)),
    command_uri_(std::move(command_uri)),
    zmq_context_(1),
    control_type_(network_interfaces::control_type_t::EFFORT),
    damping_gains_(default_damping_gains()),
    collision_behaviour_(default_collision_behaviour()) {
}

void FrankaLightWeightInterface::init() {

  this->prevTorque = Eigen::Matrix<double, 7, 1>::Zero();

  this->logFile.open("franka_lightweight_interface/analysis/data_accel.txt");


  // create connection to the robot
  this->franka_robot_ = std::make_unique<franka::Robot>(this->robot_ip_);
  this->franka_model_ = std::make_unique<franka::Model>(this->franka_robot_->loadModel());
  this->connected_ = true;

  // create zmq connections with an external controller
  // TODO: find a better way to pass in port number
  network_interfaces::zmq::configure_subscriber(this->zmq_context_, this->zmq_subscriber_, this->command_uri_, true);
  network_interfaces::zmq::configure_publisher(this->zmq_context_, this->zmq_publisher_, this->state_uri_, true);

  if (this->prefix_.empty()) {
    this->prefix_ = "franka_";
  }
  std::string robot_name = this->prefix_.substr(0, this->prefix_.length() - 1);
  std::vector<std::string> joint_names(7);
  for (std::size_t j = 0; j < joint_names.size(); ++j) {
    joint_names.at(j) = this->prefix_ + "joint" + std::to_string(j + 1);
  }
  this->state_.ee_state = state_representation::CartesianState(this->prefix_ + "ee", this->prefix_ + "base");
  this->state_.joint_state = state_representation::JointState(robot_name, joint_names);
  this->state_.jacobian =
      state_representation::Jacobian(robot_name, joint_names, this->prefix_ + "ee", this->prefix_ + "base");
  this->state_.mass =
      state_representation::Parameter<Eigen::MatrixXd>(this->prefix_ + "mass", Eigen::MatrixXd::Zero(7, 7));

  this->command_.control_type = std::vector<int>{static_cast<int>(this->control_type_)};
  this->command_.joint_state = state_representation::JointState("franka", 7);

  this->last_command_ = std::chrono::steady_clock::now();
}

void FrankaLightWeightInterface::reset_command() {
  this->command_.control_type = std::vector<int>(7, static_cast<int>(network_interfaces::control_type_t::UNDEFINED));
  this->command_.joint_state.set_velocities(Eigen::VectorXd::Zero(7));
  this->command_.joint_state.set_accelerations(Eigen::VectorXd::Zero(7));
  this->command_.joint_state.set_torques(Eigen::VectorXd::Zero(7));
}

void FrankaLightWeightInterface::set_damping_gains(const Eigen::Array<double, 7, 1>& damping_gains) {
  this->damping_gains_ = damping_gains;
}

void FrankaLightWeightInterface::set_damping_gains(const std::array<double, 7>& damping_gains) {
  this->set_damping_gains(Eigen::ArrayXd::Map(damping_gains.data(), 7));
}

void FrankaLightWeightInterface::set_collision_behaviour(
    const std::array<double, 7>& lower_torque_thresholds_acceleration,
    const std::array<double, 7>& upper_torque_thresholds_acceleration,
    const std::array<double, 7>& lower_torque_thresholds_nominal,
    const std::array<double, 7>& upper_torque_thresholds_nominal,
    const std::array<double, 6>& lower_force_thresholds_acceleration,
    const std::array<double, 6>& upper_force_thresholds_acceleration,
    const std::array<double, 6>& lower_force_thresholds_nominal,
    const std::array<double, 6>& upper_force_thresholds_nominal
) {
  this->set_collision_behaviour(
      {
          lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration, lower_torque_thresholds_nominal,
          upper_torque_thresholds_nominal, lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
          lower_force_thresholds_nominal, upper_force_thresholds_nominal
      }
  );
}

void FrankaLightWeightInterface::set_collision_behaviour(const CollisionBehaviour& collision_behaviour) {
  this->collision_behaviour_ = collision_behaviour;
}

void FrankaLightWeightInterface::run_controller() {
  if (this->is_connected()) {
    // restart the controller unless the node is shutdown
    while (!this->is_shutdown()) {
      try {
        switch (this->control_type_) {
          case network_interfaces::control_type_t::EFFORT:
            std::cout << "Starting joint torque controller..." << std::endl;
            this->run_joint_torques_controller();
            break;
          default:
            std::cout << "Unimplemented control type! (" << this->control_type_ << ")" << std::endl;
            this->control_type_ = network_interfaces::control_type_t::UNDEFINED;
            [[fallthrough]];
          case network_interfaces::control_type_t::UNDEFINED:
            std::cout << "Starting state publisher..." << std::endl;
            this->run_state_publisher();
            break;
        }
      } catch (const franka::CommandException& e) {
        std::cerr << e.what() << std::endl;
      }
      std::cerr << "Controller stopped but the node is still active, restarting..." << std::endl;
      //flush and reset any remaining command messages
      network_interfaces::zmq::receive(this->command_, this->zmq_subscriber_);
      this->reset_command();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } else {
    throw std::runtime_error("Robot not connected! Call the init function first.");
  }
}

void FrankaLightWeightInterface::poll_external_command() {
  if (network_interfaces::zmq::receive(this->command_, this->zmq_subscriber_)) {
    if (this->command_.joint_state.is_empty()) {
      throw std::runtime_error("Received joint command is empty.");
    }
    this->last_command_ = std::chrono::steady_clock::now();
    const auto& control_type = this->command_.control_type.at(0);
    for (auto type_iter = std::next(this->command_.control_type.begin());
         type_iter != this->command_.control_type.end(); ++type_iter) {
      if (*type_iter != control_type) {
        throw std::runtime_error(
            "Currently, only commands where all the joints have the same control type are supported."
        );
      }
    }
    this->control_type_ = network_interfaces::control_type_t(control_type);
  } else if (std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - this->last_command_
  ).count() > this->command_timeout_.count()) {
    this->reset_command();
  }
}

void FrankaLightWeightInterface::publish_robot_state() {
  network_interfaces::zmq::send(this->state_, this->zmq_publisher_);
}

void FrankaLightWeightInterface::read_robot_state(const franka::RobotState& robot_state) {
  // extract cartesian info
  Eigen::Affine3d eef_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  this->state_.ee_state.set_pose(eef_transform.translation(), Eigen::Quaterniond(eef_transform.linear()));
  this->state_.ee_state.set_wrench(Eigen::MatrixXd::Map(robot_state.O_F_ext_hat_K.data(), 6, 1));

  // extract joint info
  assert(robot_state.q.size() == 7);
  this->state_.joint_state.set_positions(Eigen::VectorXd::Map(robot_state.q.data(), 7));
  this->state_.joint_state.set_velocities(Eigen::VectorXd::Map(robot_state.dq.data(), 7));
  this->state_.joint_state.set_torques(Eigen::VectorXd::Map(robot_state.tau_J.data(), 7));

  // extract jacobian
  std::array<double, 42> jacobian_array = this->franka_model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  this->state_.jacobian.set_data(Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data()));

  std::array<double, 49> current_mass_array = this->franka_model_->mass(robot_state);
  this->state_.mass.set_value(Eigen::Map<const Eigen::Matrix<double, 7, 7>>(current_mass_array.data()));

  // get the twist from jacobian and current joint velocities
  this->state_.ee_state.set_twist(this->state_.jacobian * this->state_.joint_state.get_velocities());
}

void FrankaLightWeightInterface::run_state_publisher() {
  try {
    this->franka_robot_->read(
        [this](const franka::RobotState& robot_state) {
          // check the local socket for a command
          this->poll_external_command();
          if (this->control_type_ != network_interfaces::control_type_t::UNDEFINED) {
            std::cout << "Received a new control type command - switching! " << std::endl;
            return false;
          }
          this->read_robot_state(robot_state);
          this->publish_robot_state();
          return true;
        }
    );
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void FrankaLightWeightInterface::run_joint_torques_controller() {
  // Set additional parameters always before the control loop, NEVER in the control loop!

  // Set collision behavior.
  this->franka_robot_->setCollisionBehavior(
      this->collision_behaviour_.ltta, this->collision_behaviour_.utta, this->collision_behaviour_.lttn,
      this->collision_behaviour_.uttn, this->collision_behaviour_.lfta, this->collision_behaviour_.ufta,
      this->collision_behaviour_.lftn, this->collision_behaviour_.uftn
  );

  try {
    this->franka_robot_->control(
        [this](
            const franka::RobotState& robot_state, franka::Duration
        ) -> franka::Torques {
          // check the local socket for a torque command
          this->poll_external_command();

          if (this->control_type_ != network_interfaces::control_type_t::EFFORT) {
            if (this->control_type_ == network_interfaces::control_type_t::UNDEFINED) {
              throw franka::ControlException("Control type reset!");
            }
            throw IncompatibleControlTypeException("Control type changed!");
          }

          // lock mutex
          std::lock_guard<std::mutex> lock(this->get_mutex());
          // extract current state
          this->read_robot_state(robot_state);

          // get the coriolis array
          std::array<double, 7> coriolis_array = this->franka_model_->coriolis(robot_state);
          Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());

          // get the mass matrix
          std::array<double, 49> mass_array = franka_model_->mass(robot_state);
          Eigen::Map<const Eigen::Matrix<double, 7, 7> > mass(mass_array.data());

          // Get the gravity vector
          std::array<double, 7> gravity_array = this->franka_model_->gravity(robot_state);
          Eigen::Map<const Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());

          static int count = 0;
          static Eigen::Matrix<double, 7, 1> joint_target;

          std::vector<double> min_range = {-2.8973-1,	-1.7628-0.8,	-2.8973-1,	-3.0718-1.2,	-2.8973-1.5,	-0.0175,	-2.8973-1.5};
          std::vector<double> max_range = {2.8973+1, 1.7628+0.8,	2.8973+1,	-0.0698-0.6,	2.8973+1.5,	3.7525+1.5,	2.8973+1.5};

          // std::vector<double> min_range = {-2.8973,	-1.7628,	-2.8973,	-3.0718,	-2.8973,	-0.0175,	-2.8973};
          // std::vector<double> max_range = {2.8973, 1.7628,	2.8973,	-0.0698,	2.8973,	3.7525,	2.8973};

          if (count > 1000){
            std::random_device rd;  // Will be used to obtain a seed for the random number engine
            std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()

            float margin = 0.7;
            
            for(int i=0; i<7; i++){
              std::uniform_real_distribution<> dis(margin*min_range.at(i), margin*max_range.at(i));
              joint_target(i) = dis(gen);
            }

            std::cout << joint_target.transpose() << std::endl;
            count = 0;
          }
          count++;

          // PD control
          Eigen::Matrix<double, 7, 7> stiffness = Eigen::Matrix<double, 7, 7>::Zero();
          stiffness.diagonal() << 2, 3, 1.5, 2, 1.3, 1.5, 1;
          Eigen::Matrix<double, 7, 7> damping = Eigen::Matrix<double, 7, 7>::Zero();
          damping.diagonal() << 1.8, 1.65, 1.65, 1.4, 1.8, 1.6, 1.7;
          Eigen::Matrix<double, 7, 1> newTorque;
          newTorque = stiffness*(joint_target - this->state_.joint_state.get_positions()) - damping*this->state_.joint_state.get_velocities();

          // table avoidance
          Eigen::Matrix<double, 7, 1> table_repulsion = this->state_.jacobian.data().transpose().col(2);
          table_repulsion *= 10*exp(-10*pow(this->state_.ee_state.get_position()[2], 2));
          newTorque += table_repulsion;

          // Smooth out torque
          double alpha = 0.03;
          Eigen::Matrix<double, 7, 1> commandTorque;
          commandTorque = (1-alpha)*this->prevTorque + alpha*newTorque;

          this->prevTorque = commandTorque;

          // // Generate sinus acceleration
          // static double counter = 0;
          Eigen::Matrix<double, 7, 1> desired_acceleration {0, 0, 0, 1, 0, 0, 0};
          // desired_acceleration *= sin(6.14*1*counter);
          // counter += 1e-3;

          // Eigen::Matrix<double, 7, 1> commandTorque = this->state_.mass.get_value()*desired_acceleration;

  
          command_.joint_state.set_torques(commandTorque);

          // Friction model
          Eigen::Array<double, 7, 1> phi_1 {1.849, 2.21, 2.229, 1.514, 1.674, 1.538, 1.054};
          Eigen::Array<double, 7, 1> phi_2 {31.706, 110.026, 142.178, 107.409, 300, 300, 300};
          Eigen::Array<double, 7, 1> phi_3 {-0.751, -2.094, -0.875, -1.261, -0.911, -0.878, -0.527};
          Eigen::Array<double, 7, 1> phi_4 {0.895, 0.616, 0.637, 0.4, 0.308, 0.137, 0.141};
          Eigen::Array<double, 7, 1> phi_5 {-0.171, -1.053, -0.033, -0.322, -0.061, 0.028, -0.042};

          // Eigen::Array<double, 7, 1> velocity = (this->state_.joint_state.get_velocities().array().abs()<0.005).select(0.,this->state_.joint_state.get_velocities().array());
          Eigen::Array<double, 7, 1> velocity = this->state_.joint_state.get_velocities().array();
          Eigen::Array<double, 7, 1> friction_torque = phi_1/( 1+(-phi_2*velocity).exp() ) + phi_3 + phi_4*velocity + phi_5*this->state_.joint_state.get_positions().array();


          std::array<double, 7> torques{};
          Eigen::VectorXd::Map(&torques[0], 7) = this->command_.joint_state.get_torques().array()
              - this->damping_gains_ * this->state_.joint_state.get_velocities().array() + coriolis.array();// + friction_torque;

 
          Eigen::Matrix<double, 7, 1> measured_joint_torque = this->state_.joint_state.get_torques() - coriolis - gravity;

          // Write data to txt file
          this->logFile << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() << " " 
                        << this->state_.joint_state.get_positions().transpose() << " " 
                        << commandTorque.transpose() << " " 
                        << desired_acceleration.transpose() << std::endl;



          // write the state out to the local socket
          this->publish_robot_state();

          //return torques;
          return torques;
        }
    );
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}
}// namespace frankalwi
