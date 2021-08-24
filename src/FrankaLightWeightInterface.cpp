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

FrankaLightWeightInterface::FrankaLightWeightInterface(
    std::string robot_ip, std::string state_uri, std::string command_uri
) :
    robot_ip_(std::move(robot_ip)),
    connected_(false),
    shutdown_(false),
    state_uri_(std::move(state_uri)),
    command_uri_(std::move(command_uri)),
    zmq_context_(1),
    collision_behaviour_(default_collision_behaviour()) {
}

void FrankaLightWeightInterface::init() {
  // create connection to the robot
  this->franka_robot_ = std::make_unique<franka::Robot>(this->robot_ip_);
  this->franka_model_ = std::make_unique<franka::Model>(this->franka_robot_->loadModel());
  this->connected_ = true;

  // create zmq connections with an external controller
  // TODO: find a better way to pass in port number
  this->zmq_publisher_ = zmq::socket_t(this->zmq_context_, ZMQ_PUB);
  this->zmq_publisher_.connect("tcp://" + this->state_uri_);

  this->zmq_subscriber_ = zmq::socket_t(this->zmq_context_, ZMQ_SUB);
  this->zmq_subscriber_.set(zmq::sockopt::conflate, 1);
  this->zmq_subscriber_.set(zmq::sockopt::subscribe, "");
  this->zmq_subscriber_.connect("tcp://" + this->command_uri_);

  this->state_.ee_state = state_representation::CartesianState("franka_ee", "franka_base");
  this->state_.joint_state = state_representation::JointState("franka", 7);
  this->state_.jacobian = state_representation::Jacobian("franka", 7, "franka_ee", "franka_base");
  this->state_.mass = Eigen::MatrixXd::Zero(7, 7);

  this->command_.control_type = proto::ControlType::NONE;
  this->command_.ee_state = state_representation::CartesianState("franka_ee", "franka_base");
  this->command_.joint_state = state_representation::JointState("franka", 7);

  this->last_command_ = std::chrono::steady_clock::now();
}

void FrankaLightWeightInterface::reset_command() {
  this->command_.ee_state.set_twist(Eigen::VectorXd::Zero(6));
  this->command_.ee_state.set_accelerations(Eigen::VectorXd::Zero(6));
  this->command_.ee_state.set_wrench(Eigen::VectorXd::Zero(6));
  this->command_.joint_state.set_velocities(Eigen::VectorXd::Zero(7));
  this->command_.joint_state.set_accelerations(Eigen::VectorXd::Zero(7));
  this->command_.joint_state.set_torques(Eigen::VectorXd::Zero(7));
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
      });
}

void FrankaLightWeightInterface::set_collision_behaviour(const CollisionBehaviour& collision_behaviour) {
  this->collision_behaviour_ = collision_behaviour;
}

void FrankaLightWeightInterface::run_controller() {
  if (this->is_connected()) {
    // restart the controller unless the node is shutdown
    while (!this->is_shutdown()) {
      try {
        switch (this->command_.control_type) {
          case proto::ControlType::JOINT_TORQUE:
            std::cout << "Starting joint torque controller..." << std::endl;
            this->run_joint_torques_controller();
            break;
          case proto::ControlType::CARTESIAN_TWIST:
            std::cout << "Starting cartesian velocities controller..." << std::endl;
            this->run_cartesian_velocities_controller();
            break;
          default:
            std::cout << "Unimplemented control type! (" << this->command_.control_type << ")" << std::endl;
            this->command_.control_type = proto::ControlType::NONE;
            [[fallthrough]];
          case proto::ControlType::NONE:
            std::cout << "Starting state publisher..." << std::endl;
            this->run_state_publisher();
            break;
        }
      } catch (const franka::CommandException& e) {
        std::cerr << e.what() << std::endl;
      }
      std::cerr << "Controller stopped but the node is still active, restarting..." << std::endl;
      //flush and reset any remaining command messages
      network::poll(this->command_, this->zmq_subscriber_);
      this->reset_command();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } else {
    std::cerr << "Robot not connected first call the init function" << std::endl;
  }
}

void FrankaLightWeightInterface::poll_external_command() {
  if (network::poll(this->command_, this->zmq_subscriber_)) {
    this->last_command_ = std::chrono::steady_clock::now();
  } else if (std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - this->last_command_).count() > this->command_timeout_.count()) {
    this->reset_command();
  }
}

void FrankaLightWeightInterface::publish_robot_state() {
  network::send(this->state_, this->zmq_publisher_);
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
  this->state_.mass = Eigen::Map<const Eigen::Matrix<double, 7, 7> >(current_mass_array.data());

  // get the twist from jacobian and current joint velocities
  this->state_.ee_state.set_twist(this->state_.jacobian * this->state_.joint_state.get_velocities());
}

void FrankaLightWeightInterface::run_state_publisher() {
  try {
    this->franka_robot_->read(
        [this](const franka::RobotState& robot_state) {
          // check the local socket for a command
          this->poll_external_command();
          if (this->command_.control_type != proto::ControlType::NONE) {
            std::cout << "Received a new control type command - switching! " << std::endl;
            return false;
          }
          this->read_robot_state(robot_state);
          this->publish_robot_state();
          return true;
        });
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
      this->collision_behaviour_.lftn, this->collision_behaviour_.uftn);

  // Set joint damping.
  Eigen::ArrayXd d_gains = Eigen::ArrayXd(7);
  d_gains << 25.0, 25.0, 25.0, 25.0, 15.0, 15.0, 5.0;

  try {
    this->franka_robot_->control(
        [this, d_gains](
            const franka::RobotState& robot_state, franka::Duration
        ) -> franka::Torques {
          // check the local socket for a torque command
          this->poll_external_command();

          if (this->command_.control_type != proto::ControlType::JOINT_TORQUE) {
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

          std::array<double, 7> torques{};
          Eigen::VectorXd::Map(&torques[0], 7) = this->command_.joint_state.get_torques().array()
              - d_gains * this->state_.joint_state.get_velocities().array() + coriolis.array();

          // write the state out to the local socket
          this->publish_robot_state();

          //return torques;
          return torques;
        });
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void FrankaLightWeightInterface::run_cartesian_velocities_controller() {
  // Set additional parameters always before the control loop, NEVER in the control loop!

  // Set collision behavior.
  this->franka_robot_->setCollisionBehavior(
      this->collision_behaviour_.ltta, this->collision_behaviour_.utta, this->collision_behaviour_.lttn,
      this->collision_behaviour_.uttn, this->collision_behaviour_.lfta, this->collision_behaviour_.ufta,
      this->collision_behaviour_.lftn, this->collision_behaviour_.uftn);

  // Set carteisan impedance
  this->franka_robot_->setCartesianImpedance({{100, 100, 100, 10, 10, 10}});

  try {
    this->franka_robot_->control(
        [this](
            const franka::RobotState& robot_state, franka::Duration
        ) -> franka::CartesianVelocities {
          // check the local socket for a velocity command
          this->poll_external_command();

          if (this->command_.control_type != proto::ControlType::CARTESIAN_TWIST) {
            throw IncompatibleControlTypeException("Control type changed!");
          }

          // lock mutex
          std::lock_guard<std::mutex> lock(this->get_mutex());
          // extract current state
          this->read_robot_state(robot_state);

          std::array<double, 6> velocities{};
          Eigen::VectorXd::Map(&velocities[0], 6) = this->command_.ee_state.get_twist();

          // write the state out to the local socket
          this->publish_robot_state();
          return velocities;
        }, franka::ControllerMode::kCartesianImpedance, true, 10.0);
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}
}
