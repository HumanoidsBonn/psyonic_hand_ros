#include "psyonic_hand_driver/hand_serial.h"

namespace psyonic_hand_driver
{

uint8_t _computeChecksum(const uint8_t *ptr, const size_t size)
{
  uint8_t checksum = 0;
  for (size_t i = 0; i < size; i++)
  {
    checksum += ptr[i];
  }
  return ~checksum + 1;
}

std::string to_string(ControlMode mode)
{
  switch (mode)
  {
  case ControlMode::POSITION:
    return "POSITION";
  case ControlMode::VELOCITY:
    return "VELOCITY";
  case ControlMode::TORQUE:
    return "TORQUE";
  case ControlMode::VOLTAGE:
    return "VOLTAGE";
  case ControlMode::READ_ONLY:
    return "READ_ONLY";
  default:
    return "Unknown";
  }
}

std::string to_string(ReplyMode mode)
{
  switch (mode)
  {
  case ReplyMode::V1:
    return "V1";
  case ReplyMode::V2:
    return "V2";
  case ReplyMode::V3:
    return "V3";
  default:
    return "Unknown";
  }
}

std::string to_string(FormatHeader header)
{
  switch (header)
  {
  case FormatHeader::POSITION_MODE_REPLY_V1:
    return "POSITION_MODE_REPLY_V1";
  case FormatHeader::POSITION_MODE_REPLY_V2:
    return "POSITION_MODE_REPLY_V2";
  case FormatHeader::POSITION_MODE_REPLY_V3:
    return "POSITION_MODE_REPLY_V3";
  case FormatHeader::VELOCITY_MODE_REPLY_V1:
    return "VELOCITY_MODE_REPLY_V1";
  case FormatHeader::VELOCITY_MODE_REPLY_V2:
    return "VELOCITY_MODE_REPLY_V2";
  case FormatHeader::VELOCITY_MODE_REPLY_V3:
    return "VELOCITY_MODE_REPLY_V3";
  case FormatHeader::TORQUE_MODE_REPLY_V1:
    return "TORQUE_MODE_REPLY_V1";
  case FormatHeader::TORQUE_MODE_REPLY_V2:
    return "TORQUE_MODE_REPLY_V2";
  case FormatHeader::TORQUE_MODE_REPLY_V3:
    return "TORQUE_MODE_REPLY_V3";
  case FormatHeader::VOLTAGE_MODE_REPLY_V1:
    return "VOLTAGE_MODE_REPLY_V1";
  case FormatHeader::VOLTAGE_MODE_REPLY_V2:
    return "VOLTAGE_MODE_REPLY_V2";
  case FormatHeader::VOLTAGE_MODE_REPLY_V3:
    return "VOLTAGE_MODE_REPLY_V3";
  case FormatHeader::READ_ONLY_MODE_REPLY_V1:
    return "READ_ONLY_MODE_REPLY_V1";
  case FormatHeader::READ_ONLY_MODE_REPLY_V2:
    return "READ_ONLY_MODE_REPLY_V2";
  case FormatHeader::READ_ONLY_MODE_REPLY_V3:
    return "READ_ONLY_MODE_REPLY_V3";
  case FormatHeader::UPSAMPLE_THUMB_ROTATOR_ENABLE:
    return "UPSAMPLE_THUMB_ROTATOR_ENABLE";
  case FormatHeader::UPSAMPLE_THUMB_ROTATOR_DISABLE:
    return "UPSAMPLE_THUMB_ROTATOR_DISABLE";
  case FormatHeader::EXIT_API_CONTROL_MODE:
    return "EXIT_API_CONTROL_MODE";
  default:
    return "Unknown";
  }
}

std::unique_ptr<DecodedTouchSensorData> UnpackedTouchSensorData::decode() const
{
  constexpr double C1 = 121591.0;
  constexpr double C2 = 0.878894;
  auto decoded_touch_data = std::make_unique<DecodedTouchSensorData>();
  for (size_t i = 0; i < TOUCH_SENSOR_NUM_VALS; i++)
  {
    double V = static_cast<double>((*this)[i]) * 3.3 / 4096.0;
    double R = 33000.0 / V + 10000.0;
    (*decoded_touch_data)[i] = C1 / R + C2;
  }
  return decoded_touch_data;
}

std::unique_ptr<UnpackedTouchSensorData> HandReplyV1or2::unpackTouchSensorData() const
{
  static_assert(TOUCH_SENSOR_NUM_VALS == sizeof(touch_sensor_data) * 2 / 3, "Packed and unpacked touch sensor data sizes do not match");
  auto unpacked_touch_data = std::make_unique<UnpackedTouchSensorData>();
  memset(unpacked_touch_data.get(), 0, sizeof(UnpackedTouchSensorData));
  for(int bidx = TOUCH_SENSOR_NUM_VALS * 12 - 4; bidx >= 0; bidx -= 4)
  {
    int validx = bidx / 12;
    int arridx = bidx / 8;
    int shift_val = (bidx % 8);
    (*unpacked_touch_data)[validx] |= ((touch_sensor_data[arridx] >> shift_val) &0x0F) << (bidx % 12);
  }
  return unpacked_touch_data;
}

std::unique_ptr<DecodedHandReplyV1or2> HandReplyV1or2::decode() const
{
  auto decoded_reply = std::make_unique<DecodedHandReplyV1or2>();
  decoded_reply->index_position = posToRad(index_position);
  decoded_reply->middle_position = posToRad(middle_position);
  decoded_reply->ring_position = posToRad(ring_position);
  decoded_reply->pinky_position = posToRad(pinky_position);
  decoded_reply->thumb_flexor_position = posToRad(thumb_flexor_position);
  decoded_reply->thumb_rotator_position = posToRad(thumb_rotator_position);

  if (getReplyMode(header) == ReplyMode::V1)
  {
    decoded_reply->index_torque_or_velocity = currentToTorque(index_current_or_velocity);
    decoded_reply->middle_torque_or_velocity = currentToTorque(middle_current_or_velocity);
    decoded_reply->ring_torque_or_velocity = currentToTorque(ring_current_or_velocity);
    decoded_reply->pinky_torque_or_velocity = currentToTorque(pinky_current_or_velocity);
    decoded_reply->thumb_flexor_torque_or_velocity = currentToTorque(thumb_flexor_current_or_velocity);
    decoded_reply->thumb_rotator_torque_or_velocity = currentToTorque(thumb_rotator_current_or_velocity);
  }
  else
  {
    decoded_reply->index_torque_or_velocity = rotorVelToRadPerSec(index_current_or_velocity) / 649.0;
    decoded_reply->middle_torque_or_velocity = rotorVelToRadPerSec(middle_current_or_velocity) / 649.0;
    decoded_reply->ring_torque_or_velocity = rotorVelToRadPerSec(ring_current_or_velocity) / 649.0;
    decoded_reply->pinky_torque_or_velocity = rotorVelToRadPerSec(pinky_current_or_velocity) / 649.0;
    decoded_reply->thumb_flexor_torque_or_velocity = rotorVelToRadPerSec(thumb_flexor_current_or_velocity) / 649.0;
    decoded_reply->thumb_rotator_torque_or_velocity = rotorVelToRadPerSec(thumb_rotator_current_or_velocity) / 162.45;
  }

  decoded_reply->touch_sensor_data = unpackTouchSensorData()->decode();

  decoded_reply->hot_cold_status = hot_cold_status;

  return decoded_reply;
}

std::unique_ptr<DecodedHandReplyV3> HandReplyV3::decode() const
{
  auto decoded_reply = std::make_unique<DecodedHandReplyV3>();
  decoded_reply->index_position = posToRad(index_position);
  decoded_reply->middle_position = posToRad(middle_position);
  decoded_reply->ring_position = posToRad(ring_position);
  decoded_reply->pinky_position = posToRad(pinky_position);
  decoded_reply->thumb_flexor_position = posToRad(thumb_flexor_position);
  decoded_reply->thumb_rotator_position = posToRad(thumb_rotator_position);

  decoded_reply->index_torque = currentToTorque(index_current);
  decoded_reply->middle_torque = currentToTorque(middle_current);
  decoded_reply->ring_torque = currentToTorque(ring_current);
  decoded_reply->pinky_torque = currentToTorque(pinky_current);
  decoded_reply->thumb_flexor_torque = currentToTorque(thumb_flexor_current);
  decoded_reply->thumb_rotator_torque = currentToTorque(thumb_rotator_current);

  decoded_reply->index_velocity = rotorVelToRadPerSec(index_velocity) / 649.0;
  decoded_reply->middle_velocity = rotorVelToRadPerSec(middle_velocity) / 649.0;
  decoded_reply->ring_velocity = rotorVelToRadPerSec(ring_velocity) / 649.0;
  decoded_reply->pinky_velocity = rotorVelToRadPerSec(pinky_velocity) / 649.0;
  decoded_reply->thumb_flexor_velocity = rotorVelToRadPerSec(thumb_flexor_velocity) / 649.0;
  decoded_reply->thumb_rotator_velocity = rotorVelToRadPerSec(thumb_rotator_velocity) / 162.45;

  decoded_reply->hot_cold_status = hot_cold_status;

  return decoded_reply;
}

} // namespace psyonic_hand_driver