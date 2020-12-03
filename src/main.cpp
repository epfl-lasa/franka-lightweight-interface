//
// Created by eeberhard on 24/11/2020.
//

#include "franka_lightweight_interface/FrankaLightWeightInterface.hpp"

using namespace frankalwi;

int main(int, char**)
{
    std::string robot_ip = "172.16.0.2";
    FrankaLightWeightInterface flwi(robot_ip);
    flwi.init();
    flwi.run_controller();
    return 0;
}