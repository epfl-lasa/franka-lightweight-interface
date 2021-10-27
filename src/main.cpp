#include <iostream>

#include "franka_lightweight_interface/FrankaLightWeightInterface.hpp"

using namespace frankalwi;

static void set_joint_damping(const std::string& level, FrankaLightWeightInterface& flwi) {
  if (level == "off") {
    flwi.set_damping_gains({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  } else if (level == "low") {
    flwi.set_damping_gains({{1.0, 1.0, 0.9, 0.9, 0.8, 0.7, 0.6}});
  } else if (level == "medium") {
    flwi.set_damping_gains({{5.0, 5.0, 4.5, 4.5, 4.0, 3.5, 3.0}});
  } else if (level == "high") {
    flwi.set_damping_gains({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}});
  }
}

static void set_collision_behaviour(const std::string& level, FrankaLightWeightInterface& flwi) {
  if (level == "low") {
    flwi.set_collision_behaviour(
        {{80.0, 80.0, 72.0, 72.0, 64.0, 56, 48}}, {{80.0, 80.0, 72.0, 72.0, 64.0, 56, 48}},
        {{80.0, 80.0, 72.0, 72.0, 64.0, 56, 48}}, {{80.0, 80.0, 72.0, 72.0, 64.0, 56, 48}},
        {{80.0, 80.0, 80.0, 100.0, 100.0, 100.0}}, {{80.0, 80.0, 80.0, 100.0, 100.0, 100.0}},
        {{80.0, 80.0, 80.0, 100.0, 100.0, 100.0}}, {{80.0, 80.0, 80.0, 100.0, 100.0, 100.0}}
    );
  } else if (level == "medium") {
    flwi.set_collision_behaviour(
        {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}}, {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
        {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}}, {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
        {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}}, {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}},
        {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}}, {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}}
    );
  } else if (level == "high") {
    flwi.set_collision_behaviour(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}
    );
  }
}

// Return the value of a named option from the argument buffer
// (for example, `--key value` will point to "value" for supplied option "key")
char* parse_option(char** begin, char** end, const std::string& option) {
  char** itr = std::find(begin, end, option);
  if (itr != end && ++itr != end) {
    return *itr;
  }
  return nullptr;
}

int main(int argc, char** argv) {
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  std::string
      help_message = "Usage: franka_lightweight_interface robot-id prefix [--damping <level>] [--sensitivity <level>]";
  std::string robot_ip = "172.16.0.2";
  std::string state_uri = "0.0.0.0:1601";
  std::string command_uri = "0.0.0.0:1602";

  if (argc <= 2) {
    std::cerr << "Not enough input arguments. Provide at least the robot number and its prefix." << std::endl
              << help_message << std::endl;
    return 1;
  }
  if (atof(argv[1]) == 17) {
    robot_ip = "172.17.0.2";
    state_uri = "0.0.0.0:1701";
    command_uri = "0.0.0.0:1702";
  } else if (atof(argv[1]) != 16) {
    std::cerr << "This robot is unknown, choose either '16' or '17'." << std::endl << help_message << std::endl;
    return 1;
  }
  std::string prefix = argv[2];
  if (prefix.substr(prefix.length() - 1, 1) != "_") {
    std::cerr << "Please provide a prefix that ends with an underscore." << std::endl << help_message << std::endl;
    return 1;
  }

  FrankaLightWeightInterface flwi(robot_ip, state_uri, command_uri, prefix);

  char* option = parse_option(argv, argv + argc, "--joint-damping");
  if (option) {
    std::string joint_damping = std::string(option);
    if (joint_damping != "off" && joint_damping != "low" && joint_damping != "medium" && joint_damping != "high") {
      std::cerr << "Provide one of (off, low, medium, high) for option --damping" << std::endl << help_message
                << std::endl;
      return 1;
    }
    std::cout << "Using joint damping level " << joint_damping << std::endl;
    set_joint_damping(joint_damping, flwi);
  }

  option = parse_option(argv, argv + argc, "--collision-sensitivity");
  if (option) {
    std::string collision_sensitivity = std::string(option);
    if (collision_sensitivity != "low" && collision_sensitivity != "medium" && collision_sensitivity != "high") {
      std::cerr << "Provide one of (low, medium, high) for option --sensitivity" << std::endl << help_message
                << std::endl;
      return 1;
    }
    std::cout << "Using collision sensitivity level " << collision_sensitivity << std::endl;
    set_collision_behaviour(collision_sensitivity, flwi);
  }

  flwi.init();
  flwi.run_controller();
  return 0;
}