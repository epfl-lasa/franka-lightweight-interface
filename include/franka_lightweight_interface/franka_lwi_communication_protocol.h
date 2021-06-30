#pragma once

#include <array>

#define FRANKA_LWI_PROTO_VERSION (1.1)

namespace frankalwi::proto {

typedef double datatype;

enum ControlType {
  NONE = 0, JOINT_POSITION, JOINT_VELOCITY, JOINT_TORQUE, CARTESIAN_POSE, CARTESIAN_TWIST, CARTESIAN_WRENCH
};

// --- Message sub-structures --- //
template<std::size_t DOF>
struct Joints {
  datatype& operator[](std::size_t i) {
    return data[i % DOF];
  }
  inline std::array<datatype, DOF> array() const { return data; };
  std::array<datatype, DOF> data;
};

struct Vec3D {
  Vec3D() : x(0), y(0), z(0) {}
  explicit Vec3D(std::array<datatype, 3> vec) : x(vec[0]), y(vec[1]), z(vec[2]) {}
  inline std::array<datatype, 3> array() const { return std::array<datatype, 3>({x, y, z}); }
  datatype x;
  datatype y;
  datatype z;
};

// Quaternion is stored with scalar term first, followed by real vector terms (w, x, y, z)
struct Quaternion {
  Quaternion() : w(0), x(0), y(0), z(0) {}
  explicit Quaternion(std::array<datatype, 4> q) : w(q[0]), x(q[1]), y(q[2]), z(q[3]) {}
  inline std::array<datatype, 4> array() const { return std::array<datatype, 4>({w, x, y, z}); }
  inline std::array<datatype, 4> array_xyzw() const { return std::array<datatype, 4>({x, y, z, w}); }
  datatype w;
  datatype x;
  datatype y;
  datatype z;
};

struct EEPose {
  EEPose() : position(), orientation() {}
  explicit EEPose(std::array<datatype, 7> pose) :
      position({pose[0], pose[1], pose[2]}), orientation({pose[3], pose[4], pose[5], pose[6]}) {}
  inline std::array<datatype, 7> array() const {
    return std::array<datatype, 7>({
                                       position.x, position.y, position.z, orientation.w, orientation.x, orientation.y,
                                       orientation.z
                                   });
  }
  Vec3D position;
  Quaternion orientation;
};

struct EETwist {
  EETwist() : linear(), angular() {}
  explicit EETwist(std::array<datatype, 6> twist) :
      linear({twist[0], twist[1], twist[2]}), angular({twist[3], twist[4], twist[5]}) {}
  inline std::array<datatype, 6> array() const {
    return std::array<datatype, 6>({
                                       linear.x, linear.y, linear.z, angular.x, angular.y, angular.z
                                   });
  }
  Vec3D linear;
  Vec3D angular;
};

// Jacobian and mass matrix are serialised in column-major format
// (index = row + column * nrows), and (row = index % nrows), (column = floor(index / nrows))
// For a 6x7 matrix, the serial indices   0,      1,      2     ...  6,      7,      8 ...
//   map to the row/column indices as:   [0][0], [1][0], [2][0] ... [0][1], [1][1], [2][1] ...
template<std::size_t DOF> using Jacobian = std::array<datatype, 6 * DOF>;

template<std::size_t DOF> using Mass = std::array<datatype, DOF * DOF>;


// --- Message structures --- //
template<std::size_t DOF>
struct StateMessage {
  Joints<DOF> jointPosition;
  Joints<DOF> jointVelocity;
  Joints<DOF> jointTorque;
  EEPose eePose;
  EETwist eeTwist;
  EETwist eeWrench;
  Jacobian<DOF> jacobian;
  Mass<DOF> mass;
};

template<std::size_t DOF>
struct CommandMessage {
  ControlType controlType = NONE;
  Joints<DOF> jointPosition;
  Joints<DOF> jointVelocity;
  Joints<DOF> jointTorque;
  EEPose eePose;
  EETwist eeTwist;
  EETwist eeWrench;
};

}
