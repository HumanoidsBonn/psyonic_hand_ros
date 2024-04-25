#include "psyonic_hand_driver/hand_serial.h"

namespace psyonic_hand_driver
{

FormatHeader operator+(ControlMode control, ReplyMode reply)
{
  return static_cast<FormatHeader>(static_cast<uint8_t>(control) + static_cast<uint8_t>(reply));
}

ReplyMode getReplyMode(FormatHeader header)
{
  return static_cast<ReplyMode>(static_cast<uint8_t>(header) & 0x03);
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

} // namespace psyonic_hand_driver