
#pragma once

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>

#include "franka_lightweight_interface/franka_lwi_communication_protocol.h"

namespace frankalwi {
/**
 * @class FrankaRobotInterface
 * @brief Class to define an interface with the Franka panda robots
 *
 */
class FrankaLightWeightInterface {
private:
  std::string robot_ip_; ///< ip of the robot to connect to
  std::unique_ptr<franka::Robot> franka_robot_; ///< robot object to send command to
  std::unique_ptr<franka::Model> franka_model_; ///< model object of the robot
  bool connected_;
  bool shutdown_;
  std::string state_uri_; ///< URI of the socket to connect to for publishing state messages
  std::string command_uri_; ///< URI of the socket to connect to for receiving command messages
  zmq::context_t zmq_context_;
  zmq::socket_t zmq_publisher_;
  zmq::socket_t zmq_subscriber_;
  proto::CommandMessage<7> zmq_command_msg_{};
  proto::StateMessage<7> zmq_state_msg_{};
  Eigen::Vector3d current_cartesian_position_;
  Eigen::Quaterniond current_cartesian_orientation_;
  Eigen::Matrix<double, 6, 1> current_cartesian_twist_;
  Eigen::Matrix<double, 6, 1> current_cartesian_wrench_;
  Eigen::Matrix<double, 7, 1> current_joint_positions_;
  Eigen::Matrix<double, 7, 1> current_joint_velocities_;
  Eigen::Matrix<double, 7, 1> current_joint_torques_;
  Eigen::Matrix<double, 6, 7> current_jacobian_;
  Eigen::Matrix<double, 7, 7> current_mass_;
  Eigen::Matrix<double, 7, 1> command_joint_torques_;
  std::chrono::steady_clock::time_point last_command_;
  std::chrono::milliseconds command_timeout_ = std::chrono::milliseconds(500);
  std::mutex mutex_;

  void print_state() const;

  static bool send(const proto::StateMessage<7>& state);

  static bool poll(proto::CommandMessage<7>& command);

public:
  /**
   * @brief Constructor for the FrankaLightWeightInterface class
   * @param robot_ip ip adress of the robot to control
   */
  explicit FrankaLightWeightInterface(std::string robot_ip, std::string state_uri, std::string command_uri);

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
   * Getter of the mutex attribute
   * @return the mutex attribute
   */
  std::mutex& get_mutex();

  /**
   * @brief Initialize the connection to the robot
   */
  void init();

  /**
   * @brief Threaded function that run a controller based on the value in the active_controller enumeration
   */
  void run_controller();

  /**
   * @brief Poll the ZMQ socket subscription for a new joint torque command from an external controller
   */
  void poll_external_command();

  /**
  * @brief Publish robot state to the ZMQ socket for an external controller or observer to receive
  */
  void publish_robot_state();

  /**
   * @brief Read the robot state and update the published elements
   * @param robot_state the Franka robot state to read and get velues from
   */
  void read_robot_state(const franka::RobotState& robot_state);

  /**
   * @brief Run the joint torques controller
   * that reads commands from the joint torques subscription
   */
  void run_joint_torques_controller();

};

inline bool FrankaLightWeightInterface::is_connected() const {
  return this->connected_;
}

inline bool FrankaLightWeightInterface::is_shutdown() const {
  return this->shutdown_;
}

inline std::mutex& FrankaLightWeightInterface::get_mutex() {
  return this->mutex_;
}

inline void FrankaLightWeightInterface::print_state() const {
  std::cout << "Current robot cartesian state:" << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << this->current_cartesian_position_.transpose() << std::endl;
  std::cout << this->current_cartesian_orientation_.w() << ", "
            << this->current_cartesian_orientation_.vec().transpose() << std::endl;
  std::cout << this->current_cartesian_twist_.transpose() << std::endl;
  std::cout << this->current_cartesian_wrench_.transpose() << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << "Current robot joint state:" << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << this->current_joint_positions_.transpose() << std::endl;
  std::cout << this->current_joint_velocities_.transpose() << std::endl;
  std::cout << this->current_joint_torques_.transpose() << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << "Commanded torque:" << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << this->command_joint_torques_.transpose() << std::endl;
  std::cout << "####################" << std::endl;
}
}