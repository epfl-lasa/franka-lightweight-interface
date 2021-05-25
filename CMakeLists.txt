cmake_minimum_required(VERSION 3.9)

# Options. Turn on with 'cmake -Druntests=ON'.
option(runtests "Build all tests." OFF) # Makes boolean 'test' available.

project(franka_lightweight_interface)

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

find_package(Eigen3 REQUIRED)
find_package(cppzmq REQUIRED)

include_directories(
 	include
 	${Eigen3_INCLUDE_DIRS}
)

set(CORE_ROBOT_SOURCES
    src/FrankaLightWeightInterface.cpp
    src/main_robot.cpp
)
add_executable (${PROJECT_NAME} ${CORE_ROBOT_SOURCES})
target_link_libraries(${PROJECT_NAME} franka Eigen3::Eigen cppzmq)

set(CORE_GRIPPER_SOURCES
		src/FrankaGripperInterface.cpp
		src/main_gripper.cpp
)
add_executable (franka_gripper_interface ${CORE_GRIPPER_SOURCES})
target_link_libraries(franka_gripper_interface franka cppzmq)

install(DIRECTORY include/
  	DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  	ARCHIVE DESTINATION lib
  	LIBRARY DESTINATION lib
  	RUNTIME DESTINATION bin
)

