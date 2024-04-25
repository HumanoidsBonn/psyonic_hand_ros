#pragma once

#include "hand_data_conversion.h"

#include <cstdint>
#include <memory>
#include <string>

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

ReplyMode getReplyMode(FormatHeader header);

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
static constexpr size_t DECODED_TOUCH_SENSOR_DATA_SIZE = TOUCH_SENSOR_NUM_VALS * sizeof(double);

struct DecodedTouchSensorData
{
  double index_site0;
  double index_site1;
  double index_site2;
  double index_site3;
  double index_site4;
  double index_site5;
  double middle_site0;
  double middle_site1;
  double middle_site2;
  double middle_site3;
  double middle_site4;
  double middle_site5;
  double ring_site0;
  double ring_site1;
  double ring_site2;
  double ring_site3;
  double ring_site4;
  double ring_site5;
  double pinky_site0;
  double pinky_site1;
  double pinky_site2;
  double pinky_site3;
  double pinky_site4;
  double pinky_site5;
  double thumb_site0;
  double thumb_site1;
  double thumb_site2;
  double thumb_site3;
  double thumb_site4;
  double thumb_site5;

  double& operator[](size_t idx)
  {
    return reinterpret_cast<double*>(this)[idx];
  }

  const double& operator[](size_t idx) const
  {
    return reinterpret_cast<const double*>(this)[idx];
  }
};

static_assert(sizeof(DecodedTouchSensorData) == DECODED_TOUCH_SENSOR_DATA_SIZE, "DecodedTouchSensorData size is not correct");

struct UnpackedTouchSensorData
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

  uint16_t& operator[](size_t idx)
  {
    return reinterpret_cast<uint16_t*>(this)[idx];
  }

  const uint16_t& operator[](size_t idx) const
  {
    return reinterpret_cast<const uint16_t*>(this)[idx];
  }

  std::unique_ptr<DecodedTouchSensorData> decode() const;
};

static_assert(sizeof(UnpackedTouchSensorData) == TOUCH_SENSOR_DATA_SIZE, "TouchSensorData size is not correct");

struct DecodedHandReplyV1
{
  double index_position;
  double index_torque;
  double middle_position;
  double middle_torque;
  double ring_position;
  double ring_torque;
  double pinky_position;
  double pinky_torque;
  double thumb_flexor_position;
  double thumb_flexor_torque;
  double thumb_rotator_position;
  double thumb_rotator_torque;
};

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

  std::unique_ptr<UnpackedTouchSensorData> unpackTouchSensorData() const;
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

} // namespace psyonic_hand_driver