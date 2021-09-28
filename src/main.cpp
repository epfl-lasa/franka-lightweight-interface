#include <iostream>

#include "franka_lightweight_interface/FrankaLightWeightInterface.hpp"

using namespace frankalwi;

int main(int argc, char** argv) {
  std::string robot_ip = "172.16.0.2";
  std::string state_uri = "0.0.0.0:1601";
  std::string command_uri = "0.0.0.0:1602";
  std::string prefix;
  if (argc >= 2) {
    if (atof(argv[1]) == 17) {
      robot_ip = "172.17.0.2";
      state_uri = "0.0.0.0:1701";
      command_uri = "0.0.0.0:1702";
    } else if (atof(argv[1]) != 16) {
      std::cerr << "This robot is unknown, choose either '16' or '17'." << std::endl;
      return 1;
    }
  }
  if (argc == 3) {
    prefix = argv[2];
    if (prefix.substr(prefix.length() - 1, 1) != "_") {
      std::cerr << "Please provide a prefix that ends with an underscore." << std::endl;
      return 1;
    }
  } else if (argc > 3) {
    std::cerr << "Please provide at most two arguments: robot ('16' or '17') and prefix." << std::endl;
    return 1;
  }
  FrankaLightWeightInterface flwi(robot_ip, state_uri, command_uri, prefix);
  flwi.init();
  flwi.run_controller();
  return 0;
}