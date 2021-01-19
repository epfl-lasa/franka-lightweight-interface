#pragma once

#include <zmq.hpp>
#include <array>

namespace frankalwi::proto {

// --- Message sub-structures --- //
template<std::size_t DOF>
struct Joints {
  double& operator[](std::size_t i) {
    return data[i % DOF];
  }
  std::array<double, DOF> data;
};

struct Vec3D {
  Vec3D() : x(0), y(0), z(0) {}
  explicit Vec3D(std::array<double, 3> vec) : x(vec[0]), y(vec[1]), z(vec[2]) {}
  float x;
  float y;
  float z;
};

struct Quaternion {
  Quaternion() : w(0), x(0), y(0), z(0) {}
  explicit Quaternion(std::array<double, 4> q) : w(q[0]), x(q[1]), y(q[2]), z(q[3]) {}
  float w;
  float x;
  float y;
  float z;
};

struct EEPose {
  EEPose() : position(), orientation() {}
  explicit EEPose(std::array<double, 7> pose) : position({pose[0], pose[1], pose[2]}),
                                                orientation({pose[3], pose[4], pose[5], pose[6]}) {

  }
  Vec3D position;
  Quaternion orientation;
};

struct EETwist {
  EETwist() : linear(), angular() {}
  explicit EETwist(std::array<double, 6> twist) : linear({twist[0], twist[1], twist[2]}),
                                                  angular({twist[3], twist[4], twist[5]}) {}
  Vec3D linear;
  Vec3D angular;
};

template<std::size_t DOF>
using Jacobian = std::array<double, 6 * DOF>;

template<std::size_t DOF>
using Mass = std::array<double, DOF * DOF>;


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
  Joints<DOF> jointTorque;
};


// --- Communication helpers --- //

template<typename T>
bool send(zmq::socket_t& publisher, const T& obj) {
  zmq::message_t message(sizeof(obj));
  memcpy(message.data(), &obj, sizeof(obj));
  auto res = publisher.send(message, zmq::send_flags::none);

  return res.has_value();
}

template<typename T>
bool receive(zmq::socket_t& subscriber, T& obj, const zmq::recv_flags flags = zmq::recv_flags::none) {
  zmq::message_t message;
  auto res = subscriber.recv(message, flags);
  if (res) {
    obj = *message.data<T>();
  }
  return res.has_value();
}

template<typename T>
bool poll(zmq::socket_t& subscriber, T& obj) {
  return receive(subscriber, obj, zmq::recv_flags::dontwait);
}

}
