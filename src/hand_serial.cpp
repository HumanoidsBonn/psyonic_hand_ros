#include "psyonic_hand_driver/hand_serial.h"

#include <filesystem>

#include <ros/ros.h>

namespace psyonic_hand_driver
{

FormatHeader operator+(ControlMode control, ReplyMode reply)
{
  return static_cast<FormatHeader>(static_cast<uint8_t>(control) + static_cast<uint8_t>(reply));
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

std::unique_ptr<HandReply> HandSerial::PPP_unstuff(const std::vector<uint8_t> &stuffed_data, size_t expected_size)
{
  auto res = std::make_unique<HandReply>();
  uint8_t *ptr = reinterpret_cast<uint8_t*>(res.get());

  if (stuffed_data.size() < 2)
  {
    ROS_ERROR_STREAM("Invalid PPP frame: Too short");
    return nullptr;
  }
  if (stuffed_data[0] != 0x7E || stuffed_data.back() != 0x7E)
  {
    ROS_ERROR_STREAM("Invalid PPP frame: Missing start or end flag");
    return nullptr;
  }

  size_t read = 0;
  for (size_t i = 1; i < stuffed_data.size(); i++)
  {
    if (stuffed_data[i] == 0x7E)
    {
      break;
    }
    if (read >= expected_size)
    {
      ROS_ERROR_STREAM("Invalid PPP frame; more than " << expected_size << " bytes read");
      return nullptr;
    }
    if (stuffed_data[i] == 0x7D)
    {
      i++;
      if (i >= stuffed_data.size())
      {
        ROS_ERROR_STREAM("Invalid PPP frame; more than " << expected_size << " bytes read");
        return nullptr;
      }
      ptr[read++] = stuffed_data[i] ^ 0x20;
    }
    else
    {
      ptr[read++] = stuffed_data[i];
    }
  }
  if (read != expected_size)
  {
    ROS_ERROR_STREAM("Invalid PPP frame; " << read << " bytes read out of " << expected_size << " expected bytes");
    return nullptr;
  }
  return res;
}

std::unique_ptr<HandReply> HandSerial::receive(ReplyMode reply_mode, const ros::Duration &timeout)
{
  const size_t MSG_SIZE = (reply_mode == ReplyMode::V3) ? HAND_REPLY_V3_SIZE : HAND_REPLY_V1OR2_SIZE;
  const size_t BUF_SIZE = 2*MSG_SIZE + 2;
  std::vector<uint8_t> buffer(BUF_SIZE);
  size_t read_ind = 0;
  ros::Time start = ros::Time::now();
  try
  {
    while (buffer[0] != 0x7E && ros::Time::now() - start < timeout)
    {
      sp.read(buffer.data(), 1);
    }
    if (buffer[0] != 0x7E)
    {
      ROS_ERROR_STREAM("Failed to find start of PPP frame; read " << read_ind << " bytes");
      return nullptr;
    }
    read_ind = 1;
    while ((read_ind < 2 || buffer[read_ind - 1] != 0x7E) && ros::Time::now() - start < timeout)
    {
      size_t read_bytes = sp.read(buffer.data() + read_ind, BUF_SIZE - read_ind);
      read_ind += read_bytes;
    }
    if (read_ind < 2 || buffer[read_ind - 1] != 0x7E)
    {
      ROS_ERROR_STREAM("Failed to find end of PPP frame; read " << read_ind << " bytes");
      return nullptr;
    }
  }
  catch (const serial::IOException &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return nullptr;
  }
  buffer.resize(read_ind);
  auto res = PPP_unstuff(buffer, MSG_SIZE);
  if (!res)
  {
    ROS_ERROR("Failed to unstuff PPP frame");
    return nullptr;
  }
  uint8_t received_checksum = (reply_mode == ReplyMode::V3) ? res->v3.checksum : res->v1or2.checksum;
  uint8_t computed_checksum = computeChecksum(*res);
  if (computed_checksum != received_checksum)
  {
    ROS_ERROR_STREAM("Checksum mismatch");
    return nullptr;
  }
  return res;
}

std::unique_ptr<HandReply> HandSerial::queryStatus(ReplyMode reply_mode)
{
  HandMiscCommand msg;
  msg.header = ControlMode::READ_ONLY + reply_mode;

  if (!send(msg))
  {
    return nullptr;
  }

  return receive(reply_mode, ros::Duration(0.25));
}

std::unique_ptr<HandReply> HandSerial::sendPositions(ReplyMode reply_mode, double index, double middle, double ring, double pinky, double thumb_flexor, double thumb_rotator)
{
  HandControlCommand msg;
  msg.header = ControlMode::POSITION + reply_mode;
  msg.index_data = radToPos(index);
  msg.middle_data = radToPos(middle);
  msg.ring_data = radToPos(ring);
  msg.pinky_data = radToPos(pinky);
  msg.thumb_flexor_data = radToPos(thumb_flexor);
  msg.thumb_rotator_data = radToPos(thumb_rotator);

  if (!send(msg))
  {
    return nullptr;
  }

  return receive(reply_mode, ros::Duration(0.25));
}

} // namespace psyonic_hand_driver