#include "franka_lightweight_interface/FrankaLightWeightInterface.hpp"

using namespace frankalwi;

int main(int, char**) {
  std::string robot_ip = "172.16.0.2";
  std::string state_uri = "0.0.0.0:5550";
  std::string command_uri = "0.0.0.0:5551";
  FrankaLightWeightInterface flwi(robot_ip, state_uri, command_uri);
  flwi.init();
  flwi.run_controller();
  return 0;
}