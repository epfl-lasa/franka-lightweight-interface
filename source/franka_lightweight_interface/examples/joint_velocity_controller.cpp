#include "network_interfaces/control_type.h"
#include "network_interfaces/zmq/network.h"

int main(int argc, char** argv) {
  std::string state_uri = "*:1601";
  std::string command_uri = "*:1602";
  if (argc == 2) {
    if (atof(argv[1]) == 17) {
      state_uri = "*:1701";
      command_uri = "*:1702";
    } else if (atof(argv[1]) != 16) {
      std::cerr << "This robot is unknown, choose either '16' (default) or '17'." << std::endl;
      return 1;
    }
  } else {
    std::cerr << "Please provide at most one argument to choose which robot to connect to ('16' or '17')." << std::endl;
    return 1;
  }

  double gain = 0.5;

  ::zmq::context_t context(1);
  ::zmq::socket_t subscriber, publisher;
  network_interfaces::zmq::configure_sockets(context, subscriber, state_uri, publisher, command_uri, true, true);

  network_interfaces::zmq::StateMessage state;
  network_interfaces::zmq::CommandMessage command;

  command.control_type = std::vector<int>{static_cast<int>(network_interfaces::control_type_t::VELOCITY)};
  command.joint_state = state_representation::JointState::Zero("franka", 7);

  state_representation::JointPositions target;
  while (context.handle() != nullptr) {
    if (network_interfaces::zmq::receive(state, subscriber)) {
      if (target.is_empty()) {
        target = state.joint_state;
        command.joint_state = state.joint_state;
        command.joint_state.set_zero();
      }

      auto velocities = gain * (target.get_positions() - state.joint_state.get_positions());
      command.joint_state.set_velocities(velocities);

      network_interfaces::zmq::send(command, publisher);
    }
  }
}