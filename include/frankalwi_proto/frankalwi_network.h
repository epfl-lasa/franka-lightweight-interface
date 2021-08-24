#pragma once

#include <zmq.hpp>

#include "frankalwi_proto.h"

namespace frankalwi::network {

// --- Transceiving methods --- //
inline bool send(const proto::StateMessage& state, zmq::socket_t& publisher) {
  auto encoded_message = proto::encode_state(state);
  zmq::message_t message(sizeof(proto::EncodedStateMessage));
  memcpy(message.data(), &encoded_message, sizeof(encoded_message));
  auto res = publisher.send(message, zmq::send_flags::none);

  return res.has_value();
}

inline bool poll(proto::CommandMessage& command, zmq::socket_t& subscriber) {
  zmq::message_t message;
  auto res = subscriber.recv(message, zmq::recv_flags::dontwait);
  if (res) {
    auto encoded_command = *message.data<proto::EncodedCommandMessage>();
    command = proto::decode_command(encoded_command);
  }
  return res.has_value();
}

}