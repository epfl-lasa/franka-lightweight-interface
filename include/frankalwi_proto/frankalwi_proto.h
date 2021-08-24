#pragma once

#include <clproto.h>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/robot/JointState.hpp>
#include <state_representation/robot/Jacobian.hpp>
#include <state_representation/parameters/Parameter.hpp>

namespace frankalwi::proto {

// --- Message structures --- //
enum ControlType {
  NONE = 0, JOINT_POSITION, JOINT_VELOCITY, JOINT_TORQUE, CARTESIAN_POSE, CARTESIAN_TWIST, CARTESIAN_WRENCH
};

struct StateMessage {
  state_representation::CartesianState ee_state;
  state_representation::JointState joint_state;
  state_representation::Jacobian jacobian;
  Eigen::MatrixXd mass;
};

struct CommandMessage {
  ControlType control_type = NONE;
  state_representation::CartesianState ee_state;
  state_representation::JointState joint_state;
};

// --- Encoded message structures --- //
struct EncodedStateMessage {
  std::string ee_state;
  std::string joint_state;
  std::string jacobian;
  std::string mass;
};

struct EncodedCommandMessage {
  std::string control_type;
  std::string ee_state;
  std::string joint_state;
};

// --- Encoding methods --- //
inline EncodedStateMessage encode_state(const StateMessage& state) {
  EncodedStateMessage message;
  message.ee_state = clproto::encode(state.ee_state);
  message.joint_state = clproto::encode(state.joint_state);
  message.jacobian = clproto::encode(state.jacobian);
  message.mass = clproto::encode(state_representation::Parameter("mass", state.mass));
  return message;
}

inline EncodedCommandMessage encode_command(const CommandMessage& command) {
  EncodedCommandMessage message;
  message.control_type = clproto::encode(
      state_representation::Parameter<double>("control_type", static_cast<double>(command.control_type)));
  message.ee_state = clproto::encode(command.ee_state);
  message.joint_state = clproto::encode(command.joint_state);
  return message;
}

// --- Decoding methods --- //
inline StateMessage decode_state(const EncodedStateMessage& message) {
  StateMessage state;
  state.ee_state = clproto::decode<state_representation::CartesianState>(message.ee_state);
  state.joint_state = clproto::decode<state_representation::JointState>(message.joint_state);
  state.jacobian = clproto::decode<state_representation::Jacobian>(message.jacobian);
  state.mass = clproto::decode<state_representation::Parameter<Eigen::MatrixXd>>(message.mass).get_value();
  return state;
}

inline CommandMessage decode_command(const EncodedCommandMessage& message) {
  CommandMessage command;
  command.control_type = static_cast<ControlType>(clproto::decode<state_representation::Parameter<double>>(
      message.control_type).get_value());
  command.ee_state = clproto::decode<state_representation::CartesianState>(message.ee_state);
  command.joint_state = clproto::decode<state_representation::JointState>(message.joint_state);
  return command;
}

}