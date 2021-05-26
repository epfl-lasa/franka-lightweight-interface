#include "franka_lightweight_interface/FrankaGripperInterface.hpp"

#include <utility>
#include <thread>

namespace frankalwi {

FrankaGripperInterface::FrankaGripperInterface(std::string gripper_ip, std::string state_uri, std::string command_uri) :
    gripper_ip_(std::move(gripper_ip)),
    connected_(false),
    shutdown_(false),
    state_uri_(std::move(state_uri)),
    command_uri_(std::move(command_uri)),
    zmq_context_(1) {
}

void FrankaGripperInterface::init() {
  // create connection to the robot
  this->gripper_ = std::make_unique<franka::Gripper>(this->gripper_ip_);
  this->gripper_->homing();
  this->connected_ = true;

  // create zmq connections with an external controller
  this->zmq_publisher_ = zmq::socket_t(this->zmq_context_, ZMQ_PUB);
  this->zmq_publisher_.connect("tcp://" + this->state_uri_);

  this->zmq_subscriber_ = zmq::socket_t(this->zmq_context_, ZMQ_SUB);
  this->zmq_subscriber_.set(zmq::sockopt::conflate, 1);
  this->zmq_subscriber_.set(zmq::sockopt::subscribe, "");
  this->zmq_subscriber_.connect("tcp://" + this->command_uri_);
}

void FrankaGripperInterface::run_controller() {
  if (this->is_connected()) {
    // restart the controller unless the node is shutdown
    while (!this->is_shutdown()) {
      try {
        // write the state out to the local socket
        this->publish_gripper_state();

        // check the local socket for a command
        if (this->poll(this->zmq_command_msg_)) {
          bool success;
          if (this->zmq_command_msg_.stop) {
            success = this->gripper_->stop();
          } else if (this->zmq_command_msg_.home) {
            success = this->gripper_->homing();
          } else if (this->zmq_command_msg_.force == 0) {
            success = this->gripper_->move(this->zmq_command_msg_.width, this->zmq_command_msg_.speed);
          } else if (this->zmq_command_msg_.force > 0) {
            success = this->gripper_->grasp(this->zmq_command_msg_.width,
                                            this->zmq_command_msg_.speed,
                                            this->zmq_command_msg_.force,
                                            this->zmq_command_msg_.epsilon_inner,
                                            this->zmq_command_msg_.epsilon_outer);
          }
//          std::cout << success << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      } catch (const franka::Exception& e) {
        std::cerr << e.what() << std::endl;
      }
    }
  } else {
    std::cerr << "Gripper not connected, call the init function first" << std::endl;
  }
}

void FrankaGripperInterface::publish_gripper_state() {
  current_state_ = this->gripper_->readOnce();
//  print_state();
  this->zmq_state_msg_.width = current_state_.width;
  this->zmq_state_msg_.max_width = current_state_.max_width;
  this->zmq_state_msg_.is_grasped = current_state_.is_grasped;
  send(this->zmq_state_msg_);
}

bool FrankaGripperInterface::send(const proto::GripperStateMessage& state) {
  zmq::message_t message(sizeof(state));
  memcpy(message.data(), &state, sizeof(state));
  auto res = this->zmq_publisher_.send(message, zmq::send_flags::none);

  return res.has_value();
}

bool FrankaGripperInterface::poll(proto::GripperCommandMessage& command) {
  zmq::message_t message;
  auto res = this->zmq_subscriber_.recv(message, zmq::recv_flags::dontwait);
  if (res) {
    command = *message.data<proto::GripperCommandMessage>();
  }
  return res.has_value();
}
}
