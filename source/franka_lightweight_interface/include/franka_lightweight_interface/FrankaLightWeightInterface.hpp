#pragma once

#include <random>
#include <fstream>

#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>
#include <zmq.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <network_interfaces/zmq/network.h>

namespace frankalwi {

struct CollisionBehaviour {
  std::array<double, 7> ltta; ///lower_torque_thresholds_acceleration
  std::array<double, 7> utta; ///upper_torque_thresholds_acceleration
  std::array<double, 7> lttn; ///lower_torque_thresholds_nominal
  std::array<double, 7> uttn; ///upper_torque_thresholds_nominal
  std::array<double, 6> lfta; ///lower_force_thresholds_acceleration
  std::array<double, 6> ufta; ///upper_force_thresholds_acceleration
  std::array<double, 6> lftn; ///lower_force_thresholds_nominal
  std::array<double, 6> uftn; ///upper_force_thresholds_nominal
};

/**
 * @class FrankaLightweightInterface
 * @brief Class to define an interface with the Franka panda robots
 *
 */
class FrankaLightWeightInterface {
private:
  std::string prefix_; ///< prefix of the robot joints
  std::string robot_ip_; ///< ip of the robot to connect to
  std::unique_ptr<franka::Robot> franka_robot_; ///< robot object to send command to
  std::unique_ptr<franka::Model> franka_model_; ///< model object of the robot
  bool connected_;
  bool shutdown_;
  std::string state_uri_; ///< URI of the socket to connect to for publishing state messages
  std::string command_uri_; ///< URI of the socket to connect to for receiving command messages
  ::zmq::context_t zmq_context_;
  ::zmq::socket_t zmq_publisher_;
  ::zmq::socket_t zmq_subscriber_;
  network_interfaces::zmq::StateMessage state_;
  network_interfaces::zmq::CommandMessage command_;
  network_interfaces::control_type_t control_type_;
  Eigen::ArrayXd damping_gains_;
  CollisionBehaviour collision_behaviour_;
  std::chrono::steady_clock::time_point last_command_;
  std::chrono::milliseconds command_timeout_ = std::chrono::milliseconds(500);
  std::mutex mutex_;

  std::ofstream logFile;

  Eigen::Matrix<double, 7, 1> prevTorque;

  void print_state() const;

public:
  /**
   * @brief Constructor for the FrankaLightWeightInterface class
   * @param robot_ip ip address of the robot to control
   */
  explicit FrankaLightWeightInterface(
      std::string robot_ip, std::string state_uri, std::string command_uri, std::string prefix
  );

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
   * @brief Set the joint damping gains.
   * @param[in] damping_gains The desired array of damping gains per joint.
   */
  void set_damping_gains(const Eigen::Array<double, 7, 1>& damping_gains);

  /**
   * @brief Set the joint damping gains.
   * @param[in] damping_gains The desired array of damping gains per joint.
   */
  void set_damping_gains(const std::array<double, 7>& damping_gains);

  /**
   * @brief Set the collision behaviour.
   *
   * @details Set separate torque and force boundaries for acceleration/deceleration and constant velocity
   * movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause the robot to
   * stop moving.
   *
   * The new values only take effect when a controller is started.
   *
   * @param[in] lower_torque_thresholds_acceleration Contact torque thresholds during
   * acceleration/deceleration for each joint in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds_acceleration Collision torque thresholds during
   * acceleration/deceleration for each joint in \f$[Nm]\f$.
   * @param[in] lower_torque_thresholds_nominal Contact torque thresholds for each joint
   * in \f$[Nm]\f$.
   * @param[in] upper_torque_thresholds_nominal Collision torque thresholds for each joint
   * in \f$[Nm]\f$.
   * @param[in] lower_force_thresholds_acceleration Contact force thresholds during
   * acceleration/deceleration for \f$(x,y,z,R,P,Y)\f$ in \f$[N]\f$.
   * @param[in] upper_force_thresholds_acceleration Collision force thresholds during
   * acceleration/deceleration for \f$(x,y,z,R,P,Y)\f$ in \f$[N]\f$.
   * @param[in] lower_force_thresholds_nominal Contact force thresholds for \f$(x,y,z,R,P,Y)\f$
   * in \f$[N]\f$.
   * @param[in] upper_force_thresholds_nominal Collision force thresholds for \f$(x,y,z,R,P,Y)\f$
   * in \f$[N]\f$.
   */
  void set_collision_behaviour(
      const std::array<double, 7>& lower_torque_thresholds_acceleration,
      const std::array<double, 7>& upper_torque_thresholds_acceleration,
      const std::array<double, 7>& lower_torque_thresholds_nominal,
      const std::array<double, 7>& upper_torque_thresholds_nominal,
      const std::array<double, 6>& lower_force_thresholds_acceleration,
      const std::array<double, 6>& upper_force_thresholds_acceleration,
      const std::array<double, 6>& lower_force_thresholds_nominal,
      const std::array<double, 6>& upper_force_thresholds_nominal
  );

  /**
   * @brief Set the collision behaviour.
   * @copydetails FrankaLightWeightInterface::set_collision_behaviour(<!--
   * -->const std::array<double, 7>&,const std::array<double, 7>&,<!--
   * -->const std::array<double, 7>&,const std::array<double, 7>&,<!--
   * -->const std::array<double, 6>&,const std::array<double, 6>&,<!--
   * -->const std::array<double, 6>&,const std::array<double, 6>&)
   * @param collision_behaviour The collision behaviour structure
   * containing lower and upper torque and force thresholds.
   * @see FrankaLightWeightInterface::set_collision_behaviour(<!--
   * -->const std::array<double, 7>&,const std::array<double, 7>&,<!--
   * -->const std::array<double, 7>&,const std::array<double, 7>&,<!--
   * -->const std::array<double, 6>&,const std::array<double, 6>&,<!--
   * -->const std::array<double, 6>&,const std::array<double, 6>&)
   */
  void set_collision_behaviour(const CollisionBehaviour& collision_behaviour);

  /**
   * @brief Initialize the connection to the robot
   */
  void init();

  /**
   * @brief Reset the commanded state variable derivatives to zero (twists, accelerations and torques).
   */
  void reset_command();

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
   * @brief Read and publish the robot state while no control commands are received
   */
  void run_state_publisher();

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
  std::cout << this->state_.ee_state << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << "Current robot joint state:" << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << this->state_.joint_state << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << "Commanded torque:" << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << this->command_.joint_state.get_torques().transpose() << std::endl;
  std::cout << "####################" << std::endl;
}
}
