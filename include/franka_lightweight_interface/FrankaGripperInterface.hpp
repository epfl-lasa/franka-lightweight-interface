#pragma once

#include <franka/exception.h>
#include <franka/gripper.h>
#include <zmq.hpp>

#include <iostream>
#include <chrono>

#include "franka_lightweight_interface/franka_lwi_communication_protocol.h"

namespace frankalwi {
/**
 * @class FrankaGripperInterface
 * @brief Class to define an interface with the Franka panda gripper
 *
 */
class FrankaGripperInterface {
private:
  std::string gripper_ip_; ///< ip of the gripper to connect to
  std::unique_ptr<franka::Gripper> gripper_; ///< gripper object to send command to
  bool connected_;
  bool shutdown_;
  std::string state_uri_; ///< URI of the socket to connect to for publishing state messages
  std::string command_uri_; ///< URI of the socket to connect to for receiving command messages
  zmq::context_t zmq_context_;
  zmq::socket_t zmq_publisher_;
  zmq::socket_t zmq_subscriber_;
  proto::GripperCommandMessage zmq_command_msg_{};
  proto::GripperStateMessage zmq_state_msg_{};
  franka::GripperState current_state_;
  std::mutex mutex_;

  void print_state() const;

  bool send(const proto::GripperStateMessage& state);

  bool poll(proto::GripperCommandMessage& command);

public:
  /**
   * @brief Constructor for the FrankaGripperInterface class
   * @param gripper_ip ip adress of the gripper to control
   */
  explicit FrankaGripperInterface(std::string gripper_ip, std::string state_uri, std::string command_uri);

  /**
   * @brief Getter of the connected boolean attribute
   * @return the value of the connected attribute
   */
  bool is_connected() const;

  /**
   * @brief Getter of the shutdown boolean attribute
   * @return the value of the shutdown attribute
   */
  bool is_shutdown() const;

  /**
   * @brief Initialize the connection to the gripper
   */
  void init();

  /**
   * @brief Run the gripper controller that reads commands from the zmq subscription
   */
  void run_controller();

  /**
  * @brief Publish robot state to the ZMQ socket for an external controller or observer to receive
  */
  void publish_gripper_state();

};

inline bool FrankaGripperInterface::is_connected() const {
  return this->connected_;
}

inline bool FrankaGripperInterface::is_shutdown() const {
  return this->shutdown_;
}

inline void FrankaGripperInterface::print_state() const {
  std::cout << "Current gripper state:" << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << this->current_state_.width << std::endl;
  std::cout << this->current_state_.max_width << std::endl;
  std::cout << this->current_state_.is_grasped << std::endl;
}
}
