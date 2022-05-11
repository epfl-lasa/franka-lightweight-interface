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

static Eigen::Array<double, 7, 1> default_joint_damping_gains() {
  Eigen::ArrayXd gains = Eigen::ArrayXd(7);
  gains << 25.0, 25.0, 25.0, 25.0, 15.0, 15.0, 5.0;
  return gains;
}

static std::array<double, 7> default_joint_impedance_values() {
  return {2000, 2000, 2000, 1500, 1500, 1000, 1000};
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
    control_type_(network_interfaces::control_type_t::UNDEFINED),
    joint_damping_gains_(default_joint_damping_gains()),
    joint_impedance_values_(default_joint_impedance_values()),
    collision_behaviour_(default_collision_behaviour()) {
    previous_torque(Eigen::Vector<double, 7>::Zero()),
    joint_friction(Eigen::Vector<double, 7>::Zero()) {
}

void FrankaLightWeightInterface::init() {
  // create connection to the robot
  this->franka_robot_ = std::make_unique<franka::Robot>(this->robot_ip_);
  this->franka_model_ = std::make_unique<franka::Model>(this->franka_robot_->loadModel());
  this->connected_ = true;

  // create zmq connections with an external controller
  // TODO: find a better way to pass in port number
  network_interfaces::zmq::configure_subscriber(this->zmq_context_, this->zmq_subscriber_, this->command_uri_, false);
  network_interfaces::zmq::configure_publisher(this->zmq_context_, this->zmq_publisher_, this->state_uri_, false);

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

void FrankaLightWeightInterface::set_joint_damping(const Eigen::Array<double, 7, 1>& joint_damping_gains) {
  this->joint_damping_gains_ = joint_damping_gains;
}

void FrankaLightWeightInterface::set_joint_damping(const std::array<double, 7>& joint_damping_gains) {
  this->set_joint_damping(Eigen::ArrayXd::Map(joint_damping_gains.data(), 7));
}

void FrankaLightWeightInterface::set_joint_impedance(const Eigen::Array<double, 7, 1>& joint_impedance_values) {
  std::array<double, 7> values{};
  for (std::size_t i = 0; i < 7; ++i) {
    values.at(i) = joint_impedance_values(i);
  }
  this->set_joint_impedance(values);
}

void FrankaLightWeightInterface::set_joint_impedance(const std::array<double, 7>& joint_impedance_values) {
  this->joint_impedance_values_ = joint_impedance_values;
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
          case network_interfaces::control_type_t::VELOCITY:
            std::cout << "Starting joint velocity controller..." << std::endl;
            this->run_joint_velocities_controller();
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

void FrankaLightWeightInterface::run_joint_velocities_controller() {
  // Set additional parameters always before the control loop, NEVER in the control loop!

  this->franka_robot_->setJointImpedance(this->joint_impedance_values_);

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
        ) -> franka::JointVelocities {
          // check the local socket for a velocity command
          this->poll_external_command();

          if (this->control_type_ != network_interfaces::control_type_t::VELOCITY) {
            if (this->control_type_ == network_interfaces::control_type_t::UNDEFINED) {
              throw franka::ControlException("Control type reset!");
            }
            throw IncompatibleControlTypeException("Control type changed!");
          }

          // lock mutex
          std::lock_guard<std::mutex> lock(this->get_mutex());
          // extract current state
          this->read_robot_state(robot_state);

          std::array<double, 7> velocities{};
          Eigen::VectorXd::Map(&velocities[0], 7) = this->command_.joint_state.get_velocities().array();

          // write the state out to the local socket
          this->publish_robot_state();

          //return velocities;
          return velocities;
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

          // Estimate friction
          double alpha = 0.02;
          Eigen::Vector<double, 7> new_friction = this->previous_torque - this->state_.joint_state.get_torques() + gravity;
          Eigen::Vector<double, 7> filtered_friction = alpha*new_friction + (1-alpha)*this->joint_friction;
          this->joint_friction = filtered_friction;
          double friction_compensation = 0.0;
          if (this->command_.joint_state.get_velocities().isZero()) friction_compensation = 0.3;
          else friction_compensation = 1.0;
          Eigen::Vector<double, 7> friction_correction = friction_compensation*this->joint_friction;
          // std::cout << friction_correction.transpose() << std::endl;

          Eigen::Vector<double, 7> torque_command = this->command_.joint_state.get_torques().array()
              - this->joint_damping_gains_ * this->state_.joint_state.get_velocities().array() + coriolis.array() + friction_correction.array();

          this->previous_torque = torque_command;


          std::array<double, 7> torques{};
          Eigen::VectorXd::Map(&torques[0], 7) = torque_command.array() ;

          // write the state out to the local socket
          this->publish_robot_state();

          return torques;
        }
    );
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}
}// namespace frankalwi
