#include "franka_lightweight_interface/FrankaLightWeightInterface.hpp"

#include <utility>

class IncompatibleControlTypeException : public std::runtime_error {
public:
  explicit IncompatibleControlTypeException(const std::string& msg) : runtime_error(msg) {};
};

namespace frankalwi {
FrankaLightWeightInterface::FrankaLightWeightInterface(std::string robot_ip,
                                                       std::string state_uri,
                                                       std::string command_uri) :
    robot_ip_(std::move(robot_ip)),
    connected_(false),
    shutdown_(false),
    state_uri_(std::move(state_uri)),
    command_uri_(std::move(command_uri)),
    zmq_context_(1) {
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

  this->current_cartesian_twist_.setZero();
  this->current_cartesian_wrench_.setZero();
  this->current_joint_positions_.setZero();
  this->current_joint_velocities_.setZero();
  this->current_joint_torques_.setZero();
  this->command_joint_torques_.setZero();

  this->last_command_ = std::chrono::steady_clock::now();
}

void FrankaLightWeightInterface::run_controller() {
  if (this->is_connected()) {
    // restart the controller unless the node is shutdown
    while (!this->is_shutdown()) {
      try {
        switch (this->control_type_) {
          case proto::JOINT_TORQUE:
            std::cout << "Starting joint torque controller..." << std::endl;
            this->run_joint_torques_controller();
            break;
          case proto::CARTESIAN_TWIST:
            std::cout << "Starting cartesian velocities controller..." << std::endl;
            this->run_cartesian_velocities_controller();
            break;
          default:
            std::cout << "Unimplemented control type! (" << this->control_type_ << ")" << std::endl;
            [[fallthrough]];
          case proto::NONE:
            std::cout << "Starting state publisher..." << std::endl;
            this->run_state_publisher();
            break;
        }
      }
      catch (const franka::CommandException& e) {
        std::cerr << e.what() << std::endl;
      }
      std::cerr << "Controller stopped but the node is still active, restarting..." << std::endl;
      //flush and reset any remaining command messages
      poll(this->zmq_command_msg_);
      this->command_joint_torques_.setZero();
      this->command_cartesian_twist_.setZero();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } else {
    std::cerr << "Robot not connected first call the init function" << std::endl;
  }
}

void FrankaLightWeightInterface::poll_external_command() {
  if (poll(this->zmq_command_msg_)) {
    this->last_command_ = std::chrono::steady_clock::now();
    this->control_type_ = this->zmq_command_msg_.controlType;

    this->command_joint_positions_ = Eigen::Map<Eigen::MatrixXd>(this->zmq_command_msg_.jointPosition.array().data(), 7, 1);
    this->command_joint_velocities_ = Eigen::Map<Eigen::MatrixXd>(this->zmq_command_msg_.jointVelocity.array().data(), 7, 1);
    this->command_joint_torques_ = Eigen::Map<Eigen::MatrixXd>(this->zmq_command_msg_.jointTorque.array().data(), 7, 1);

    this->command_cartesian_position_ = Eigen::Vector3d(this->zmq_command_msg_.eePose.position.array().data());
    this->command_cartesian_orientation_ = Eigen::Quaterniond(this->zmq_command_msg_.eePose.orientation.array_xyzw().data());
    this->command_cartesian_twist_.block<3, 1>(0, 0) = Eigen::Vector3d(this->zmq_command_msg_.eeTwist.linear.array().data());
    this->command_cartesian_twist_.block<3, 1>(3, 0) = Eigen::Vector3d(this->zmq_command_msg_.eeTwist.angular.array().data());
    this->command_cartesian_wrench_.block<3, 1>(0, 0) = Eigen::Vector3d(this->zmq_command_msg_.eeWrench.linear.array().data());
    this->command_cartesian_wrench_.block<3, 1>(3, 0) = Eigen::Vector3d(this->zmq_command_msg_.eeWrench.angular.array().data());
  } else if (std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - this->last_command_).count() > this->command_timeout_.count()) {
    this->command_joint_velocities_.setZero();
    this->command_joint_torques_.setZero();
    this->command_cartesian_twist_.setZero();
    this->command_cartesian_wrench_.setZero();
  }
}

void FrankaLightWeightInterface::publish_robot_state() {
  Eigen::MatrixXd::Map(this->zmq_state_msg_.jointPosition.data.data(), 7, 1) = this->current_joint_positions_.array();
  Eigen::MatrixXd::Map(this->zmq_state_msg_.jointVelocity.data.data(), 7, 1) = this->current_joint_velocities_.array();
  Eigen::MatrixXd::Map(this->zmq_state_msg_.jointTorque.data.data(), 7, 1) = this->current_joint_torques_.array();

  Eigen::MatrixXd::Map(this->zmq_state_msg_.jacobian.data(), 6, 7) = this->current_jacobian_.array();
  Eigen::MatrixXd::Map(this->zmq_state_msg_.mass.data(), 7, 7) = this->current_mass_.array();

  std::array<double, 7> eePose{};
  Eigen::MatrixXd::Map(&eePose[0], 3, 1) = this->current_cartesian_position_.array();
  eePose[3] = this->current_cartesian_orientation_.w();
  Eigen::MatrixXd::Map(&eePose[4], 3, 1) = this->current_cartesian_orientation_.vec().array();
  this->zmq_state_msg_.eePose = frankalwi::proto::EEPose(eePose);

  std::array<double, 6> eeTwist{};
  Eigen::MatrixXd::Map(&eeTwist[0], 6, 1) = this->current_cartesian_twist_.array();
  this->zmq_state_msg_.eeTwist = frankalwi::proto::EETwist(eeTwist);

  Eigen::MatrixXd::Map(&eeTwist[0], 6, 1) = this->current_cartesian_wrench_.array();
  this->zmq_state_msg_.eeWrench = frankalwi::proto::EETwist(eeTwist);

  send(this->zmq_state_msg_);
}

void FrankaLightWeightInterface::read_robot_state(const franka::RobotState& robot_state) {
  // extract cartesian info
  Eigen::Affine3d eef_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  this->current_cartesian_position_ = eef_transform.translation();
  this->current_cartesian_orientation_ = Eigen::Quaterniond(eef_transform.linear());
  this->current_cartesian_wrench_ =
      Eigen::VectorXd::Map(robot_state.O_F_ext_hat_K.data(), robot_state.O_F_ext_hat_K.size());

  // extract joint info
  this->current_joint_positions_ = Eigen::VectorXd::Map(robot_state.q.data(), robot_state.q.size());
  this->current_joint_velocities_ = Eigen::VectorXd::Map(robot_state.dq.data(), robot_state.q.size());
  this->current_joint_torques_ = Eigen::VectorXd::Map(robot_state.tau_J.data(), robot_state.q.size());

  // extract jacobian
  std::array<double, 42> jacobian_array = this->franka_model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  this->current_jacobian_ = Eigen::Map<const Eigen::Matrix<double, 6, 7> >(jacobian_array.data());

  std::array<double, 49> current_mass_array = this->franka_model_->mass(robot_state);
  this->current_mass_ = Eigen::Map<const Eigen::Matrix<double, 7, 7> >(current_mass_array.data());

  // get the twist from jacobian and current joint velocities
  this->current_cartesian_twist_ = this->current_jacobian_ * this->current_joint_velocities_;
}


void FrankaLightWeightInterface::run_state_publisher() {
  try {
    this->franka_robot_->read([this](const franka::RobotState& robot_state) {
      // check the local socket for a command
      this->poll_external_command();
      if (this->control_type_ != proto::NONE) {
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
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

  // Damping
  Eigen::ArrayXd d_gains = Eigen::ArrayXd(7);
  d_gains << 25.0, 25.0, 25.0, 25.0, 15.0, 15.0, 5.0;

  try {
    this->franka_robot_->control([this, d_gains](const franka::RobotState& robot_state,
                                                 franka::Duration) -> franka::Torques {
      // check the local socket for a torque command
      this->poll_external_command();

      if (this->control_type_ != proto::JOINT_TORQUE) {
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
      Eigen::VectorXd::Map(&torques[0], 7) = this->command_joint_torques_.array()
          - d_gains * current_joint_velocities_.array()
          + coriolis.array();

      // write the state out to the local socket
      this->publish_robot_state();

      //return torques;
      return torques;
    });
  }
  catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void FrankaLightWeightInterface::run_cartesian_velocities_controller() {
  // Set additional parameters always before the control loop, NEVER in the control loop!

  this->franka_robot_->setCartesianImpedance({{100, 100, 100, 10, 10, 10}});

  // Set collision behavior.
  this->franka_robot_->setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

  try {
    this->franka_robot_->control([this](const franka::RobotState& robot_state,
                                                 franka::Duration) -> franka::CartesianVelocities {
      // check the local socket for a velocity command
      this->poll_external_command();

      if (this->control_type_ != proto::CARTESIAN_TWIST) {
        throw IncompatibleControlTypeException("Control type changed!");
      }

      // lock mutex
      std::lock_guard<std::mutex> lock(this->get_mutex());
      // extract current state
      this->read_robot_state(robot_state);

      std::array<double, 6> velocities{};
      Eigen::VectorXd::Map(&velocities[0], 6) = this->command_cartesian_twist_.array();

      // write the state out to the local socket
      this->publish_robot_state();
      return velocities;
    }, franka::ControllerMode::kCartesianImpedance, true, 10.0);
  }
  catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

bool FrankaLightWeightInterface::send(const proto::StateMessage<7>& state) {
  zmq::message_t message(sizeof(state));
  memcpy(message.data(), &state, sizeof(state));
  auto res = this->zmq_publisher_.send(message, zmq::send_flags::none);

  return res.has_value();
}

bool FrankaLightWeightInterface::poll(proto::CommandMessage<7>& command) {
  zmq::message_t message;
  auto res = this->zmq_subscriber_.recv(message, zmq::recv_flags::dontwait);
  if (res) {
    command = *message.data<proto::CommandMessage<7>>();
  }
  return res.has_value();
}
}
