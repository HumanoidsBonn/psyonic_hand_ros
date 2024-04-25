#pragma once

#include "hand_data_conversion.h"
#include "hand_serial_data.h"

#include <vector>
#include <string>
#include <optional>
#include <memory>

#include <serial/serial.h>

#include <ros/ros.h>

namespace psyonic_hand_driver
{

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