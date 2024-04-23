#include "psyonic_hand_driver/hand_serial.h"

#include <filesystem>

#include <ros/ros.h>

namespace psyonic_hand_driver
{

std::string to_string(FormatHeader header)
{
  switch (header)
  {
  case FormatHeader::POSITION_MODE_V1:
    return "POSITION_MODE_V1";
  case FormatHeader::POSITION_MODE_V2:
    return "POSITION_MODE_V2";
  case FormatHeader::POSITION_MODE_V3:
    return "POSITION_MODE_V3";
  case FormatHeader::VELOCITY_MODE_V1:
    return "VELOCITY_MODE_V1";
  case FormatHeader::VELOCITY_MODE_V2:
    return "VELOCITY_MODE_V2";
  case FormatHeader::VELOCITY_MODE_V3:
    return "VELOCITY_MODE_V3";
  case FormatHeader::TORQUE_MODE_V1:
    return "TORQUE_MODE_V1";
  case FormatHeader::TORQUE_MODE_V2:
    return "TORQUE_MODE_V2";
  case FormatHeader::TORQUE_MODE_V3:
    return "TORQUE_MODE_V3";
  case FormatHeader::VOLTAGE_MODE_V1:
    return "VOLTAGE_MODE_V1";
  case FormatHeader::VOLTAGE_MODE_V2:
    return "VOLTAGE_MODE_V2";
  case FormatHeader::VOLTAGE_MODE_V3:
    return "VOLTAGE_MODE_V3";
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
    return "UNKNOWN: " + static_cast<int>(header);
  }
}

std::unique_ptr<TouchSensorDataUnion> HandReplyV1or2::unpackTouchSensorData() const
{
  static_assert(TOUCH_SENSOR_NUM_VALS == sizeof(touch_sensor_data) * 2 / 3, "Packed and unpacked touch sensor data sizes do not match");
  auto unpacked_touch_data = std::make_unique<TouchSensorDataUnion>();
  for(int bidx = TOUCH_SENSOR_NUM_VALS * 12 - 4; bidx >= 0; bidx -= 4)
  {
    int validx = bidx / 12;
    int arridx = bidx / 8;
    int shift_val = (bidx % 8);
    unpacked_touch_data->values[validx] |= ((touch_sensor_data[arridx] >> shift_val) &0x0F) << (bidx % 12);
  }
  return unpacked_touch_data;
}

HandSerial::HandSerial()
{
}

HandSerial::~HandSerial()
{
}

// from: https://stackoverflow.com/a/73708874
std::vector<std::string> HandSerial::getSerialPorts()
{
  std::vector<std::string> port_names;

  std::filesystem::path p("/dev/serial/by-id");
  try
  {
    if (!std::filesystem::exists(p))
    {
      ROS_ERROR_STREAM(p.generic_string() + " does not exist");
      return port_names;
    }
    for (auto de : std::filesystem::directory_iterator(p))
    {
      if (std::filesystem::is_symlink(de.symlink_status()))
      {
        std::filesystem::path symlink_points_at = std::filesystem::read_symlink(de);
        std::filesystem::path canonical_path = std::filesystem::canonical(p / symlink_points_at);
        port_names.push_back(canonical_path.generic_string());
      }
    }
  }
  catch (const std::filesystem::filesystem_error &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return port_names;
  }
  std::sort(port_names.begin(), port_names.end());
  return port_names;
}

std::optional<std::string> HandSerial::getSerialPort(const std::string &id)
{
  std::filesystem::path p("/dev/serial/by-id/" + id);
  try
  {
    if (!std::filesystem::exists(p))
    {
      ROS_ERROR_STREAM(p.generic_string() + " does not exist");
      return std::nullopt;
    }
    if (!std::filesystem::is_symlink(p))
    {
      ROS_ERROR_STREAM(p.generic_string() + " is not a symlink");
      return std::nullopt;
    }
    auto symlink_points_at = std::filesystem::read_symlink(p);
    auto canonical_path = std::filesystem::canonical(p.parent_path() / symlink_points_at);
    return canonical_path.generic_string();
  }
  catch (const std::filesystem::filesystem_error &ex)
  {
    ROS_ERROR_STREAM(ex.what());
  }
  return std::nullopt;
}

bool HandSerial::connect(const std::string &id)
{
  auto port = getSerialPort(id);
  if (!port)
  {
    return false;
  }
  try
  {
    sp.setPort(*port);
    sp.setBaudrate(460800);
    sp.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);
    sp.open();
  }
  catch (const serial::IOException &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }
  return true;
}

std::unique_ptr<HandReplyV1or2> HandSerial::queryStatusV1()
{
  HandMiscCommand msg;
  msg.header = FormatHeader::READ_ONLY_MODE_REPLY_V1;

  if (!send(msg))
  {
    return nullptr;
  }

  return receive<HandReplyV1or2>(ros::Duration(0.25));
}

} // namespace psyonic_hand_driver