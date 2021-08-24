#pragma once

#include <zmq.hpp>

#include "frankalwi_proto.h"

namespace frankalwi::network {

// --- Configuration method --- //

/**
 * @brief Configure the zmq sockets to publish command messages and receive state messages.
 * @param context The ZMQ context object
 * @param[in,out] state_subscriber The subscription socket that is used to receive state messages
 * @param subscriber_uri The URI (IP:Port) of the frankalwi network state socket
 * @param[in,out] command_publisher The publication socket that is used to send command messages
 * @param publisher_uri The URI (IP:Port) of the frankalwi network command socket
 */
inline void configure_sockets(
    zmq::context_t& context, zmq::socket_t& state_subscriber, const std::string& subscriber_uri,
    zmq::socket_t& command_publisher, const std::string& publisher_uri
) {
  state_subscriber = zmq::socket_t(context, ZMQ_SUB);
  state_subscriber.set(zmq::sockopt::conflate, 1);
  state_subscriber.set(zmq::sockopt::subscribe, "");
  state_subscriber.bind("tcp://" + subscriber_uri);

  command_publisher = zmq::socket_t(context, ZMQ_PUB);
  command_publisher.bind("tcp://" + publisher_uri);
}

// --- Transceiving methods --- //

/**
 * @brief Send a state message.
 * @param state The StateMessage to publish
 * @param publisher The configured ZMQ publisher socket
 * @return True if the state was published, false otherwise
 */
inline bool send(const proto::StateMessage& state, zmq::socket_t& publisher) {
  auto encoded_message = proto::encode_state(state);
  zmq::message_t message(sizeof(proto::EncodedStateMessage));
  memcpy(message.data(), &encoded_message, sizeof(encoded_message));
  auto res = publisher.send(message, zmq::send_flags::none);

  return res.has_value();
}

/**
 * @brief Send a command message.
 * @param command The CommandMessage to publish
 * @param publisher The configured ZMQ publisher socket
 * @return True if the command was published, false otherwise
 */
inline bool send(const proto::CommandMessage& command, zmq::socket_t& publisher) {
  auto encoded_message = proto::encode_command(command);
  zmq::message_t message(sizeof(proto::EncodedCommandMessage));
  memcpy(message.data(), &encoded_message, sizeof(encoded_message));
  auto res = publisher.send(message, zmq::send_flags::none);

  return res.has_value();
}

/**
 * @brief Poll the socket for a state message.
 * @param[out] state The StateMessage object to modify by reference if a message is available
 * @param subscriber The configured ZMQ subscriber socket
 * @return True if a state was received, false otherwise
 */
inline bool poll(proto::StateMessage& state, zmq::socket_t& subscriber) {
  zmq::message_t message;
  auto res = subscriber.recv(message, zmq::recv_flags::dontwait);
  if (res) {
    auto encoded_state = *message.data<proto::EncodedStateMessage>();
    state = proto::decode_state(encoded_state);
  }
  return res.has_value();
}

/**
 * @brief Poll the socket for a command message.
 * @param[out] command The CommandMessage object to modify by reference if a message is available
 * @param subscriber The configured ZMQ subscriber socket
 * @return True if a command was received, false otherwise
 */
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