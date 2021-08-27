#pragma once

#include <clproto.h>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/JointState.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/parameters/Parameter.hpp>

namespace frankalwi::proto {

// --- Message structures --- //

/**
 * @enum ControlType
 * @brief An enumeration of the possible control types in Cartesian and joint space.
 */
enum ControlType {
  NONE = 0, JOINT_POSITION, JOINT_VELOCITY, JOINT_TORQUE, CARTESIAN_POSE, CARTESIAN_TWIST, CARTESIAN_WRENCH
};

/**
 * @struct StateMessage
 * @brief A collection of state variables in Cartesian and joint space that the robot publishes.
 */
struct StateMessage {
  state_representation::CartesianState ee_state;
  state_representation::JointState joint_state;
  state_representation::Jacobian jacobian;
  Eigen::MatrixXd mass;
};

/**
 * @struct CommandMessage
 * @brief A collection of state variables in Cartesian and joint space that the robot
 * follows according to the specified control type.
 * @see ControlType
 */
struct CommandMessage {
  ControlType control_type = NONE;
  state_representation::CartesianState ee_state;
  state_representation::JointState joint_state;
};

// --- Encoding methods --- //

/**
 * @brief Encode a state message into the serialized binary format.
 * @param state The StateMessage to encode
 * @return An ordered vector of encoded strings representing the state message fields
 */
inline std::vector<std::string> encode_state(const StateMessage& state) {
  std::vector<std::string> encoded_state;
  encoded_state.emplace_back(clproto::encode(state.ee_state));
  encoded_state.emplace_back(clproto::encode(state.joint_state));
  encoded_state.emplace_back(clproto::encode(state.jacobian));
  encoded_state.emplace_back(clproto::encode(state_representation::Parameter("mass", state.mass)));
  return encoded_state;
}

/**
 * @brief Encode a command message into the serialized binary format.
 * @param state The CommandMessage to encode
 * @return An ordered vector of encoded strings representing the command message fields
 */
inline std::vector<std::string> encode_command(const CommandMessage& command) {
  std::vector<std::string> encoded_command;
  encoded_command.emplace_back(clproto::encode(
      state_representation::Parameter<double>("control_type", static_cast<double>(command.control_type))));
  encoded_command.emplace_back(clproto::encode(command.ee_state));
  encoded_command.emplace_back(clproto::encode(command.joint_state));
  return encoded_command;
}

// --- Decoding methods --- //

/**
 * @brief Decode a state message from the serialized binary format.
 * @param message An ordered vector of encoded strings representing the state message fields
 * @return The equivalent StateMessage
 */
inline StateMessage decode_state(const std::vector<std::string>& message) {
  StateMessage state;
  state.ee_state = clproto::decode<state_representation::CartesianState>(message.at(0));
  state.joint_state = clproto::decode<state_representation::JointState>(message.at(1));
  state.jacobian = clproto::decode<state_representation::Jacobian>(message.at(2));
  state.mass = clproto::decode<state_representation::Parameter<Eigen::MatrixXd>>(message.at(3)).get_value();
  return state;
}

/**
 * @brief Decode a command message from the serialized binary format.
 * @param message An ordered vector of encoded strings representing the state message fields
 * @return The equivalent CommandMessage
 */
inline CommandMessage decode_command(const std::vector<std::string>& message) {
  CommandMessage command;
  command.control_type = static_cast<ControlType>(clproto::decode<state_representation::Parameter<double>>(
      message.at(0)
  ).get_value());
  command.ee_state = clproto::decode<state_representation::CartesianState>(message.at(1));
  command.joint_state = clproto::decode<state_representation::JointState>(message.at(2));
  return command;
}

}