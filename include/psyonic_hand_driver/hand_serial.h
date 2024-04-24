#pragma once

#include "hand_data_conversion.h"

#include <vector>
#include <string>
#include <optional>
#include <memory>

#include <serial/serial.h>

#include <ros/ros.h>

namespace psyonic_hand_driver
{

enum class ControlMode : uint8_t
{
  POSITION = 0x10,
  VELOCITY = 0x20,
  TORQUE = 0x30,
  VOLTAGE = 0x40,
  READ_ONLY = 0xA0
};

std::string to_string(ControlMode mode);

enum class ReplyMode : uint8_t
{
  V1 = 0x00,
  V2 = 0x01,
  V3 = 0x02
};

std::string to_string(ReplyMode mode);

enum class FormatHeader : uint8_t
{
  POSITION_MODE_REPLY_V1 = 0x10,
  POSITION_MODE_REPLY_V2 = 0x11,
  POSITION_MODE_REPLY_V3 = 0x12,
  VELOCITY_MODE_REPLY_V1 = 0x20,
  VELOCITY_MODE_REPLY_V2 = 0x21,
  VELOCITY_MODE_REPLY_V3 = 0x22,
  TORQUE_MODE_REPLY_V1 = 0x30,
  TORQUE_MODE_REPLY_V2 = 0x31,
  TORQUE_MODE_REPLY_V3 = 0x32,
  VOLTAGE_MODE_REPLY_V1 = 0x40,
  VOLTAGE_MODE_REPLY_V2 = 0x41,
  VOLTAGE_MODE_REPLY_V3 = 0x42,
  READ_ONLY_MODE_REPLY_V1 = 0xA0,
  READ_ONLY_MODE_REPLY_V2 = 0xA1,
  READ_ONLY_MODE_REPLY_V3 = 0xA2,
  UPSAMPLE_THUMB_ROTATOR_ENABLE = 0xC2,
  UPSAMPLE_THUMB_ROTATOR_DISABLE = 0xC3,
  EXIT_API_CONTROL_MODE = 0x7C
};

std::string to_string(FormatHeader header);

FormatHeader operator+(ControlMode control, ReplyMode reply);

struct __attribute__((__packed__)) HandControlCommand
{
  uint8_t slave_address = 0x50;
  FormatHeader header;
  int16_t index_data;
  int16_t middle_data;
  int16_t ring_data;
  int16_t pinky_data;
  int16_t thumb_flexor_data;
  int16_t thumb_rotator_data;
  uint8_t checksum;
};

static constexpr size_t HAND_CONTROL_COMMAND_SIZE = 15;

static_assert(sizeof(HandControlCommand) == HAND_CONTROL_COMMAND_SIZE, "HandControlCommand size is not correct");

struct __attribute__((__packed__)) HandMiscCommand
{
  uint8_t slave_address = 0x50;
  FormatHeader header;
  uint8_t checksum;
};

static constexpr size_t HAND_MISC_COMMAND_SIZE = 3;

static_assert(sizeof(HandMiscCommand) == HAND_MISC_COMMAND_SIZE, "HandMiscCommand size is not correct");

static constexpr size_t TOUCH_SENSOR_NUM_VALS = 30;
static constexpr size_t TOUCH_SENSOR_DATA_SIZE = TOUCH_SENSOR_NUM_VALS * 2; // 16 bits per value
static constexpr size_t TOUCH_SENSOR_PACKED_SIZE = TOUCH_SENSOR_NUM_VALS * 3 / 2; // 12 bits per value

struct TouchSensorData
{
  uint16_t index_site0;
  uint16_t index_site1;
  uint16_t index_site2;
  uint16_t index_site3;
  uint16_t index_site4;
  uint16_t index_site5;
  uint16_t middle_site0;
  uint16_t middle_site1;
  uint16_t middle_site2;
  uint16_t middle_site3;
  uint16_t middle_site4;
  uint16_t middle_site5;
  uint16_t ring_site0;
  uint16_t ring_site1;
  uint16_t ring_site2;
  uint16_t ring_site3;
  uint16_t ring_site4;
  uint16_t ring_site5;
  uint16_t pinky_site0;
  uint16_t pinky_site1;
  uint16_t pinky_site2;
  uint16_t pinky_site3;
  uint16_t pinky_site4;
  uint16_t pinky_site5;
  uint16_t thumb_site0;
  uint16_t thumb_site1;
  uint16_t thumb_site2;
  uint16_t thumb_site3;
  uint16_t thumb_site4;
  uint16_t thumb_site5;
};

union TouchSensorDataUnion
{
  TouchSensorData by_name;
  uint16_t values[TOUCH_SENSOR_NUM_VALS];
};

static_assert(sizeof(TouchSensorData) == TOUCH_SENSOR_DATA_SIZE, "TouchSensorData size is not correct");

static_assert(sizeof(TouchSensorDataUnion) == TOUCH_SENSOR_DATA_SIZE, "TouchSensorDataUnion size is not correct");

// V1: motor current; V2: motor rotor velocity
struct __attribute__((__packed__)) HandReplyV1or2
{
  FormatHeader header;
  int16_t index_position;
  int16_t index_current_or_velocity;
  int16_t middle_position;
  int16_t middle_current_or_velocity;
  int16_t ring_position;
  int16_t ring_current_or_velocity;
  int16_t pinky_position;
  int16_t pinky_current_or_velocity;
  int16_t thumb_flexor_position;
  int16_t thumb_flexor_current_or_velocity;
  int16_t thumb_rotator_position;
  int16_t thumb_rotator_current_or_velocity;
  uint8_t touch_sensor_data[TOUCH_SENSOR_PACKED_SIZE];
  uint8_t hot_cold_status;
  uint8_t checksum;

  std::unique_ptr<TouchSensorDataUnion> unpackTouchSensorData() const;
};

static constexpr size_t HAND_REPLY_V1OR2_SIZE = 72;

static_assert(sizeof(HandReplyV1or2) == HAND_REPLY_V1OR2_SIZE, "HandReplyV1or2 size is not correct");

struct __attribute__((__packed__)) HandReplyV3
{
  FormatHeader header;
  int16_t index_position;
  int16_t index_current;
  int16_t middle_position;
  int16_t middle_current;
  int16_t ring_position;
  int16_t ring_current;
  int16_t pinky_position;
  int16_t pinky_current;
  int16_t thumb_flexor_position;
  int16_t thumb_flexor_current;
  int16_t thumb_rotator_position;
  int16_t thumb_rotator_current;
  int16_t index_velocity;
  int16_t middle_velocity;
  int16_t ring_velocity;
  int16_t pinky_velocity;
  int16_t thumb_flexor_velocity;
  int16_t thumb_rotator_velocity;
  uint8_t hot_cold_status;
  uint8_t checksum;
};

static constexpr size_t HAND_REPLY_V3_SIZE = 39;

static_assert(sizeof(HandReplyV3) == HAND_REPLY_V3_SIZE, "HandReplyV3 size is not correct");

union HandReply
{
  HandReplyV1or2 v1or2;
  HandReplyV3 v3;
};

template<typename T>
uint8_t computeChecksum(T &msg)
{
  uint8_t checksum = 0;
  uint8_t *ptr = reinterpret_cast<uint8_t*>(&msg);
  constexpr size_t PAYLOAD_SIZE = sizeof(T) - 1; // exclude checksum field
  for (size_t i = 0; i < PAYLOAD_SIZE; i++)
  {
    checksum += ptr[i];
  }
  return ~checksum + 1;
}

class HandSerial
{
private:
  serial::Serial sp;

  std::vector<std::string> getSerialPorts();
  std::optional<std::string> getSerialPort(const std::string &id);

  template<typename T>
  std::vector<uint8_t> PPP_stuff(const T &msg);

  std::unique_ptr<HandReply> PPP_unstuff(const std::vector<uint8_t> &data, size_t expected_size);

  template<typename T>
  bool send(T &msg);

  std::unique_ptr<HandReply> receive(ReplyMode reply_mode, const ros::Duration &timeout);

public:
  HandSerial();
  ~HandSerial();

  bool connect(const std::string &id);

  std::unique_ptr<HandReply> queryStatus(ReplyMode reply_mode);

  std::unique_ptr<HandReply> sendPositions(ReplyMode reply_mode, double index, double middle, double ring, double pinky, double thumb_flexor, double thumb_rotator);
};

template<typename T>
std::vector<uint8_t> HandSerial::PPP_stuff(const T &msg)
{
  constexpr size_t UNSTUFFED_SIZE = sizeof(T);
  std::vector<uint8_t> data;
  data.reserve(UNSTUFFED_SIZE * 2 + 2);
  data.push_back(0x7E);
  const uint8_t *ptr = reinterpret_cast<const uint8_t*>(&msg);
  for (size_t i = 0; i < UNSTUFFED_SIZE; i++)
  {
    if (ptr[i] == 0x7E || ptr[i] == 0x7D)
    {
      data.push_back(0x7D);
      data.push_back(ptr[i] ^ 0x20);
    }
    else
    {
      data.push_back(ptr[i]);
    }
  }
  data.push_back(0x7E);
  return data;
}

template<typename T>
bool HandSerial::send(T &msg)
{
  uint8_t checksum = computeChecksum(msg);
  msg.checksum = checksum;
  std::vector<uint8_t> data = PPP_stuff(msg);
  size_t written = 0;
  try
  {
    written = sp.write(data);
  }
  catch (const serial::IOException &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }
  if (written != data.size())
  {
    ROS_ERROR_STREAM("Failed to write all bytes; wrote " << written << " bytes out of " << data.size() << " bytes");
    return false;
  }
  return true;
}

} // namespace psyonic_hand_driver