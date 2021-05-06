#include <iostream>

#include "franka_lightweight_interface/FrankaLightWeightInterface.hpp"

using namespace frankalwi;

int main(int argc, char** argv) {
  std::string robot_ip = "172.16.0.2";
  std::string state_uri = "0.0.0.0:1601";
  std::string command_uri = "0.0.0.0:1602";
  if (argc == 2 && atof(argv[1]) != 16) {
    if (atof(argv[1]) == 17) {
      robot_ip = "172.17.0.2";
      state_uri = "0.0.0.0:1701";
      command_uri = "0.0.0.0:1702";
    } else {
      std::cerr << "This robot is unknown, choose either '16' (default) or '17'." << std::endl;
      return 1;
    }
  } else if (argc > 2) {
    std::cerr << "Please provide at most one argument ('16' (default) or '17')." << std::endl;
    return 1;
  }
  FrankaLightWeightInterface flwi(robot_ip, state_uri, command_uri);
  flwi.init();
  flwi.run_controller();
  return 0;
}