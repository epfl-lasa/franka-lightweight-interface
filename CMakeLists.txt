cmake_minimum_required(VERSION 3.9)
project(franka_lightweight_interface VERSION 1.1)

# Options. Turn on with 'cmake -Druntests=ON'.
option(runtests "Build all tests." OFF) # Makes boolean 'test' available.


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

find_package(Franka 0.7.0 REQUIRED)
find_package(cppzmq REQUIRED)
find_library(protobuf REQUIRED)
find_library(clproto REQUIRED)
find_library(state_representation REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
 	include
 	${Eigen3_INCLUDE_DIRS}
	/usr/local/include/eigen3/
)

set(CORE_SOURCES
    src/FrankaLightWeightInterface.cpp
    src/main.cpp
)

add_executable(${PROJECT_NAME} ${CORE_SOURCES})

target_link_libraries(${PROJECT_NAME} franka protobuf clproto state_representation cppzmq)

install(DIRECTORY include/frankalwi_proto/
  	DESTINATION include/frankalwi_proto
)

install(TARGETS ${PROJECT_NAME}
  	ARCHIVE DESTINATION lib
  	LIBRARY DESTINATION lib
  	RUNTIME DESTINATION bin
)

