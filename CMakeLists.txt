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

include(FindProtobuf)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)
find_package(cppzmq REQUIRED)
find_library(clproto NAMES libclproto.so REQUIRED)
find_library(protobuf NAMES libprotobuf.so REQUIRED)
find_library(state_representation 4.0.0 NAMES libstate_representation.so REQUIRED)

include_directories(
		include
		${Eigen3_INCLUDE_DIRS}
		/usr/local/include/eigen3/
		lib/network-interfaces/cpp/include
)

set(CORE_SOURCES
	src/FrankaLightWeightInterface.cpp
	src/main.cpp
)

add_executable(${PROJECT_NAME} ${CORE_SOURCES})
target_link_libraries(${PROJECT_NAME} franka clproto cppzmq ${PROTOBUF_LIBRARY} state_representation)

install(TARGETS ${PROJECT_NAME}
		RUNTIME DESTINATION bin
)

add_executable(example_joint_position_controller examples/joint_position_controller.cpp)
target_link_libraries(example_joint_position_controller clproto cppzmq ${PROTOBUF_LIBRARY} state_representation)
