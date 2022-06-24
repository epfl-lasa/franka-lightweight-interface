cmake_minimum_required(VERSION 3.9)
project(franka_lightweight_interface VERSION 3.0)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  	set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(cppzmq REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.8.0 REQUIRED)
find_library(clproto REQUIRED)
find_library(network_interfaces REQUIRED)
find_library(state_representation REQUIRED)

include_directories(
		include
)

set(CORE_SOURCES
	src/FrankaLightWeightInterface.cpp
	src/main.cpp
)

add_executable(${PROJECT_NAME} ${CORE_SOURCES})
target_link_libraries(${PROJECT_NAME} franka clproto cppzmq state_representation)

install(TARGETS ${PROJECT_NAME}
		RUNTIME DESTINATION bin
)

add_executable(example_joint_position_controller examples/joint_position_controller.cpp)
target_link_libraries(example_joint_position_controller clproto cppzmq state_representation)

add_executable(example_joint_velocity_controller examples/joint_velocity_controller.cpp)
target_link_libraries(example_joint_velocity_controller clproto cppzmq state_representation)
