#pragma once

#include <zmq.hpp>
#include <array>

namespace frankalwi {
namespace proto {

// --- Message sub-structures --- //
template<std::size_t DOF>
struct Joints {
  float& operator[](std::size_t i) {
    return data[i % DOF];
  }
  std::array<float, DOF> data;
};

struct Vec3D {
  float x;
  float y;
  float z;
};

struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

struct EEPose {
  Vec3D position{};
  Quaternion orientation{};
};

struct EETwist {
  Vec3D linear{};
  Vec3D angular{};
};


// --- Message structures --- //
template<std::size_t DOF>
struct StateMessage {
  Joints<DOF> jointPosition;
  Joints<DOF> jointVelocity;
  Joints<DOF> jointTorque;
  EEPose eePose;
  EETwist eeTwist;
};

template<std::size_t DOF>
struct CommandMessage {
  Joints<DOF> jointTorque;
};


// --- Communication helpers --- //

zmq::socket_t bindPublisher(zmq::context_t& context, const char* address) {
  zmq::socket_t publisher(context, ZMQ_PUB);
  publisher.bind(address);
  return publisher;
}

zmq::socket_t connectSubscriber(zmq::context_t& context, const char* address) {
  zmq::socket_t subscriber(context, ZMQ_SUB);
  subscriber.set(zmq::sockopt::conflate, 1);
  subscriber.set(zmq::sockopt::subscribe, "");
  subscriber.connect(address);
  return subscriber;
}

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
}