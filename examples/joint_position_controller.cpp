#include "frankalwi_proto/frankalwi_network.h"

int main(int, char**) {
  double gain = 10.0;
  double damping = 0.1;

  zmq::context_t context(1);
  zmq::socket_t subscriber;
  zmq::socket_t publisher;
  frankalwi::network::configure_sockets(context, subscriber, "0.0.0.0:1701", publisher, "0.0.0.0:1702");

  frankalwi::proto::StateMessage state;
  frankalwi::proto::CommandMessage command;

  command.control_type = frankalwi::proto::ControlType::JOINT_TORQUE;
  command.ee_state = state_representation::CartesianState::Identity("franka_ee", "franka_base");
  command.joint_state = state_representation::JointState::Zero("franka", 7);

  state_representation::JointPositions target;
  while (context.handle() != nullptr) {
    if (frankalwi::network::poll(state, subscriber)) {
      if (target.is_empty()) {
        target = state.joint_state;
        command.ee_state = state.ee_state;
        command.ee_state.set_zero();
        command.joint_state = state.joint_state;
        command.joint_state.set_zero();
      }

      auto error = target.get_positions() - state.joint_state.get_positions();

      auto torques = gain * error - damping * state.joint_state.get_velocities();
      command.joint_state.set_torques(torques);

      frankalwi::network::send(command, publisher);
    }
  }

}