#include "psyonic_hand_driver/hand_serial.h"

#include <filesystem>

#include <ros/ros.h>

namespace psyonic_hand_driver
{

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

bool HandSerial::disconnect()
{
  try
  {
    sp.close();
  }
  catch(const serial::IOException &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }
  return true;
}

std::unique_ptr<HandReply> HandSerial::PPP_unstuff(const std::vector<uint8_t> &stuffed_data)
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
    if (read >= HAND_REPLY_SIZE)
    {
      ROS_ERROR_STREAM("Invalid PPP frame; more than " << HAND_REPLY_SIZE << " bytes read");
      return nullptr;
    }
    if (stuffed_data[i] == 0x7D)
    {
      i++;
      if (i >= stuffed_data.size())
      {
        ROS_ERROR_STREAM("Invalid PPP frame; more than " << HAND_REPLY_SIZE << " bytes read");
        return nullptr;
      }
      ptr[read++] = stuffed_data[i] ^ 0x20;
    }
    else
    {
      ptr[read++] = stuffed_data[i];
    }
  }
  if (read != res->expectedSize())
  {
    ROS_ERROR_STREAM("Invalid PPP frame; " << read << " bytes read out of " << res->expectedSize() << " expected bytes");
    return nullptr;
  }
  return res;
}

std::unique_ptr<HandReply> HandSerial::receive(const ros::Duration &timeout)
{
  constexpr size_t BUF_SIZE = 2*HAND_REPLY_SIZE + 2;
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
      size_t read_bytes = sp.read(buffer.data() + read_ind, 1); // read 1 byte at a time
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
  auto res = PPP_unstuff(buffer);
  if (!res)
  {
    ROS_ERROR("Failed to unstuff PPP frame");
    return nullptr;
  }
  if (res->getChecksum() != res->computeChecksum())
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

  return receive(ros::Duration(0.25));
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

  return receive(ros::Duration(0.25));
}

std::unique_ptr<HandReply> HandSerial::sendVelocities(ReplyMode reply_mode, double index, double middle, double ring, double pinky, double thumb_flexor, double thumb_rotator)
{
  HandControlCommand msg;
  msg.header = ControlMode::VELOCITY + reply_mode;
  msg.index_data = radPerSecToFingerVel(index);
  msg.middle_data = radPerSecToFingerVel(middle);
  msg.ring_data = radPerSecToFingerVel(ring);
  msg.pinky_data = radPerSecToFingerVel(pinky);
  msg.thumb_flexor_data = radPerSecToFingerVel(thumb_flexor);
  msg.thumb_rotator_data = radPerSecToFingerVel(thumb_rotator);

  if (!send(msg))
  {
    return nullptr;
  }

  return receive(ros::Duration(0.25));
}

std::unique_ptr<HandReply> HandSerial::sendTorque(ReplyMode reply_mode, double index, double middle, double ring, double pinky, double thumb_flexor, double thumb_rotator)
{
  HandControlCommand msg;
  msg.header = ControlMode::TORQUE + reply_mode;
  msg.index_data = torqueToCurrent(index);
  msg.middle_data = torqueToCurrent(middle);
  msg.ring_data = torqueToCurrent(ring);
  msg.pinky_data = torqueToCurrent(pinky);
  msg.thumb_flexor_data = torqueToCurrent(thumb_flexor);
  msg.thumb_rotator_data = torqueToCurrent(thumb_rotator);

  if (!send(msg))
  {
    return nullptr;
  }

  return receive(ros::Duration(0.25));
}

std::unique_ptr<HandReply> HandSerial::sendVoltage(ReplyMode reply_mode, double index, double middle, double ring, double pinky, double thumb_flexor, double thumb_rotator)
{
  HandControlCommand msg;
  msg.header = ControlMode::VOLTAGE + reply_mode;
  msg.index_data = dutyCycleToVoltage(index);
  msg.middle_data = dutyCycleToVoltage(middle);
  msg.ring_data = dutyCycleToVoltage(ring);
  msg.pinky_data = dutyCycleToVoltage(pinky);
  msg.thumb_flexor_data = dutyCycleToVoltage(thumb_flexor);
  msg.thumb_rotator_data = dutyCycleToVoltage(thumb_rotator);

  if (!send(msg))
  {
    return nullptr;
  }

  return receive(ros::Duration(0.25));
}

} // namespace psyonic_hand_driver