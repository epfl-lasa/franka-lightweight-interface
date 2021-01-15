#include "franka_lightweight_interface/FrankaLightWeightInterface.hpp"

#include <utility>

namespace frankalwi {
FrankaLightWeightInterface::FrankaLightWeightInterface(std::string robot_ip,
                                                       std::string state_uri,
                                                       std::string command_uri) :
    robot_ip_(std::move(robot_ip)),
    state_uri_(std::move(state_uri)),
    command_uri_(std::move(command_uri)),
    connected_(false),
    shutdown_(false),
    zmq_context_(1) {}

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
      std::cout << "Starting controller..." << std::endl;
      try {
        this->run_joint_torques_controller();
      }
      catch (const franka::CommandException& e) {
        std::cerr << e.what() << std::endl;
      }
      std::cerr << "Controller stopped but the node is still active, restarting..." << std::endl;
      //flush and reset any remaining command messages
      proto::poll(this->zmq_subscriber_, this->zmq_command_msg_);
      this->command_joint_torques_.setZero();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } else {
    std::cerr << "Robot not connected first call the init function" << std::endl;
  }
}

void FrankaLightWeightInterface::poll_external_command() {
  if (proto::poll(this->zmq_subscriber_, this->zmq_command_msg_)) {
    //TODO: use eigen map to copy std::array command message onto eigen matrix
    for (std::size_t joint = 0; joint < 7; ++joint) {
      this->command_joint_torques_[joint] = this->zmq_command_msg_.jointTorque[joint];
    }
    this->last_command_ = std::chrono::steady_clock::now();
  } else if (std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - this->last_command_).count() > this->command_timeout_.count()) {
    this->command_joint_torques_.setZero();
  }
}

void FrankaLightWeightInterface::publish_robot_state() {
  //TODO: use eigen map and / or custom mapping utils to simplify this
  for (std::size_t joint = 0; joint < 7; ++joint) {
    this->zmq_state_msg_.jointPosition[joint] = this->current_joint_positions_[joint];
    this->zmq_state_msg_.jointVelocity[joint] = this->current_joint_velocities_[joint];
    this->zmq_state_msg_.jointTorque[joint] = this->current_joint_torques_[joint];

    for (std::size_t dof = 0; dof < 6; ++dof) {
      this->zmq_state_msg_.jacobian[dof][joint] = this->current_jacobian_(dof, joint);
    }
  }
  std::copy(this->current_mass_array_.begin(), this->current_mass_array_.end(), this->zmq_state_msg_.mass.begin());

  this->zmq_state_msg_.eePose.position.x = this->current_cartesian_position_.x();
  this->zmq_state_msg_.eePose.position.y = this->current_cartesian_position_.y();
  this->zmq_state_msg_.eePose.position.z = this->current_cartesian_position_.z();
  this->zmq_state_msg_.eePose.orientation.w = this->current_cartesian_orientation_.w();
  this->zmq_state_msg_.eePose.orientation.x = this->current_cartesian_orientation_.x();
  this->zmq_state_msg_.eePose.orientation.y = this->current_cartesian_orientation_.y();
  this->zmq_state_msg_.eePose.orientation.z = this->current_cartesian_orientation_.z();

  this->zmq_state_msg_.eeTwist.linear.x = this->current_cartesian_twist_[0];
  this->zmq_state_msg_.eeTwist.linear.y = this->current_cartesian_twist_[1];
  this->zmq_state_msg_.eeTwist.linear.z = this->current_cartesian_twist_[2];
  this->zmq_state_msg_.eeTwist.angular.x = this->current_cartesian_twist_[3];
  this->zmq_state_msg_.eeTwist.angular.y = this->current_cartesian_twist_[4];
  this->zmq_state_msg_.eeTwist.angular.z = this->current_cartesian_twist_[5];

  this->zmq_state_msg_.eeWrench.linear.x = this->current_cartesian_wrench_[0];
  this->zmq_state_msg_.eeWrench.linear.y = this->current_cartesian_wrench_[1];
  this->zmq_state_msg_.eeWrench.linear.z = this->current_cartesian_wrench_[2];
  this->zmq_state_msg_.eeWrench.angular.x = this->current_cartesian_wrench_[3];
  this->zmq_state_msg_.eeWrench.angular.y = this->current_cartesian_wrench_[4];
  this->zmq_state_msg_.eeWrench.angular.z = this->current_cartesian_wrench_[5];

  proto::send(this->zmq_publisher_, this->zmq_state_msg_);
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

  this->current_mass_array_ = this->franka_model_->mass(robot_state);
  this->current_mass_ = Eigen::Map<const Eigen::Matrix<double, 7, 7> >(this->current_mass_array_.data());

  // get the twist from jacobian and current joint velocities
  this->current_cartesian_twist_ = this->current_jacobian_ * this->current_joint_velocities_;

//  this->print_state();
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
}