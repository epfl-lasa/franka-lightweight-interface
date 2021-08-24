#pragma once

#include <zmq.hpp>

#include "frankalwi_proto.h"

#define FRANKALWI_NETWORK_MAX_FIELD_LENGTH (4096)
#define FRANKALWI_NETWORK_MAX_FIELDS (64)

namespace frankalwi::network {

typedef std::size_t field_length_t;

// --- Serialization methods --- //

/**
 * @brief Pack an ordered vector of encoded field messages into a single data array.
 * @details To send multiple messages in one packet, there must
 * be some delimiting logic to distinguish the end of one field from the
 * start of the next. This packing function encodes the number of fields (N)
 * as the first data entry in the packet, then the size of each field in the
 * next N data entries, followed by the raw concatenated data of each field.
 * The order of the original vector is preserved. The corresponding
 * ::unpack_fields() method can be used to restore the original vector
 * of fields from the data buffer.
 * @param fields An ordered vector of encoded message fields
 * @param[out] data A raw data array to be packed with the fields
 */
inline void pack_fields(const std::vector<std::string>& fields, char* data) {
  std::size_t index = 0;
  field_length_t nfields;
  field_length_t sizes[FRANKALWI_NETWORK_MAX_FIELDS];

  // write out the number of fields
  nfields = static_cast<field_length_t>(fields.size());
  memcpy(data, &nfields, sizeof(field_length_t));
  index += sizeof(field_length_t);

  // write out the data size of each field
  for(std::size_t field = 0; field < nfields; ++field) {
    sizes[field] = static_cast<field_length_t>(fields.at(field).size());
  }
  memcpy(&data[index], sizes, nfields * sizeof(field_length_t));
  index += nfields * sizeof(field_length_t);

  // write out each field
  for(std::size_t field = 0; field < nfields; ++field) {
    memcpy(&data[index], fields.at(field).c_str(), sizes[field]);
    index += sizes[field];
  }
}

/**
 * @brief Unpack a data array into an ordered vector of encoded field messages.
 * @details A buffer of encoded fields serialized by ::pack_fields()
 * can be unpacked by this method. It expects the first data entry
 * in the data buffer to contain the number of fields (N). The next
 * N data entries then must contain the data length of each subsequent
 * field. Finally, the rest of the data is broken into ordered fields
 * based on the interpreted field data lengths.
 * @param data A raw data array that has been packed by ::pack_fields()
 * @return An ordered vector of encoded message fields
 */
inline std::vector<std::string> unpack_fields(const char* data) {
  std::size_t index = 0;
  field_length_t nfields;
  field_length_t sizes[FRANKALWI_NETWORK_MAX_FIELDS];
  char field_buffer[FRANKALWI_NETWORK_MAX_FIELD_LENGTH];
  std::vector<std::string> fields;

  // read out the number of fields
  memcpy(&nfields, data, sizeof(field_length_t));
  index += sizeof(field_length_t);

  // read out the data size of each field
  memcpy(sizes, &data[index], nfields * sizeof(field_length_t));
  index += nfields * sizeof(field_length_t);

  // read out each field
  for(std::size_t field = 0; field < nfields; ++field) {
    memcpy(field_buffer, &data[index], sizes[field]);
    fields.emplace_back(std::string(field_buffer, sizes[field]));
    index += sizes[field];
  }
  return fields;
}

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
 * @brief Send a sequence of encoded field messages.
 * @param fields An ordered vector of encoded fields
 * @param publisher The configured ZMQ publisher socket
 * @return True if the state was published, false otherwise
 */
inline bool send(const std::vector<std::string>& fields, zmq::socket_t& publisher) {
  zmq::message_t message(fields.size() * FRANKALWI_NETWORK_MAX_FIELD_LENGTH);
  pack_fields(fields, static_cast<char *>(message.data()));
  auto res = publisher.send(message, zmq::send_flags::none);
  return res.has_value();
}

/**
 * @brief Send a state message.
 * @param state The StateMessage to publish
 * @param publisher The configured ZMQ publisher socket
 * @return True if the state was published, false otherwise
 */
inline bool send(const proto::StateMessage& state, zmq::socket_t& publisher) {
  return send(proto::encode_state(state), publisher);
}

/**
 * @brief Send a command message.
 * @param command The CommandMessage to publish
 * @param publisher The configured ZMQ publisher socket
 * @return True if the command was published, false otherwise
 */
inline bool send(const proto::CommandMessage& command, zmq::socket_t& publisher) {
  return send(proto::encode_command(command), publisher);
}

/**
 * @brief Poll the socket for a sequence of encoded field messages.
 * @param[out] fields A vector of encoded fields to modify by reference if a message is available
 * @param subscriber The configured ZMQ subscriber socket
 * @return True if a state was received, false otherwise
 */
inline bool poll(std::vector<std::string>& fields, zmq::socket_t& subscriber) {
  zmq::message_t message;
  auto res = subscriber.recv(message, zmq::recv_flags::dontwait);
  if (res) {
    fields = unpack_fields(static_cast<const char*>(message.data()));
  }
  return res.has_value();
}

/**
 * @brief Poll the socket for a state message.
 * @param[out] state The StateMessage object to modify by reference if a message is available
 * @param subscriber The configured ZMQ subscriber socket
 * @return True if a state was received, false otherwise
 */
inline bool poll(proto::StateMessage& state, zmq::socket_t& subscriber) {
  std::vector<std::string> fields;
  auto res = poll(fields, subscriber);
  if (res) {
    state = proto::decode_state(fields);
  }
  return res;
}

/**
 * @brief Poll the socket for a command message.
 * @param[out] command The CommandMessage object to modify by reference if a message is available
 * @param subscriber The configured ZMQ subscriber socket
 * @return True if a command was received, false otherwise
 */
inline bool poll(proto::CommandMessage& command, zmq::socket_t& subscriber) {
  std::vector<std::string> fields;
  auto res = poll(fields, subscriber);
  if (res) {
    command = proto::decode_command(fields);
  }
  return res;
}

}