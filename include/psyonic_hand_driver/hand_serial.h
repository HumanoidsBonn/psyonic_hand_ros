#pragma once

#include <vector>
#include <string>
#include <optional>
#include <memory>

#include <serial/serial.h>

#include <ros/ros.h>

namespace psyonic_hand_driver
{

enum class FormatHeader : uint8_t
{
  POSITION_MODE_V1 = 0x10,
  POSITION_MODE_V2 = 0x11,
  POSITION_MODE_V3 = 0x12,
  VELOCITY_MODE_V1 = 0x20,
  VELOCITY_MODE_V2 = 0x21,
  VELOCITY_MODE_V3 = 0x22,
  TORQUE_MODE_V1 = 0x30,
  TORQUE_MODE_V2 = 0x31,
  TORQUE_MODE_V3 = 0x32,
  VOLTAGE_MODE_V1 = 0x40,
  VOLTAGE_MODE_V2 = 0x41,
  VOLTAGE_MODE_V3 = 0x42,
  READ_ONLY_MODE_REPLY_V1 = 0xA0,
  READ_ONLY_MODE_REPLY_V2 = 0xA1,
  READ_ONLY_MODE_REPLY_V3 = 0xA2,
  UPSAMPLE_THUMB_ROTATOR_ENABLE = 0xC2,
  UPSAMPLE_THUMB_ROTATOR_DISABLE = 0xC3,
  EXIT_API_CONTROL_MODE = 0x7C
};

std::string to_string(FormatHeader header);

// Position data: pos = angle_degrees * 32767 / 150
// Finger velocity = vel_degrees_per_sec * 32767 / 3000
// Rotor veleocity = vel_rad_per_sec * 4

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

struct __attribute__((__packed__)) TouchSensorData
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

static constexpr size_t TOUCH_SENSOR_NUM_VALS = 30;
static constexpr size_t TOUCH_SENSOR_DATA_SIZE = TOUCH_SENSOR_NUM_VALS * 2; // 16 bits per value
static constexpr size_t TOUCH_SENSOR_PACKED_SIZE = TOUCH_SENSOR_NUM_VALS * 3 / 2; // 12 bits per value

static_assert(sizeof(TouchSensorData) == TOUCH_SENSOR_DATA_SIZE, "TouchSensorData size is not correct");

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

  std::unique_ptr<TouchSensorData> unpackTouchSensorData() const;
};

static constexpr size_t HAND_REPLY_V1OR2_SIZE = 72;

static_assert(sizeof(HandReplyV1or2) == HAND_REPLY_V1OR2_SIZE, "HandReplyV1or2 size is not correct");

// V1: motor current; V2: motor rotor velocity
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

template<typename T>
uint8_t computeChecksum(T &msg)
{
  uint8_t checksum = 0;
  uint8_t *ptr = reinterpret_cast<uint8_t*>(&msg);
  size_t size = sizeof(T) - 1; // exclude checksum field
  for (size_t i = 0; i < size; i++)
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

  template<typename T>
  std::unique_ptr<T> PPP_unstuff(const std::vector<uint8_t> &data);

  template<typename T>
  bool send(T &msg);

  template<typename T>
  std::unique_ptr<T> receive(const ros::Duration &timeout);

public:
  HandSerial();
  ~HandSerial();

  double posToDegrees(int16_t pos) { return pos * 150.0 / 32767.0;}
  int16_t degreesToPos(double degrees ) { return static_cast<int16_t>(degrees * 32767.0 / 150.0); }
  double posToRad(int16_t pos) { return pos * M_PI / 39320.4;}
  int16_t radToPos(double rad) { return static_cast<int16_t>(rad * 39320.4 / M_PI); }

  bool connect(const std::string &id);

  std::unique_ptr<HandReplyV1or2> queryStatusV1();
};

template<typename T>
std::vector<uint8_t> HandSerial::PPP_stuff(const T &msg)
{
  std::vector<uint8_t> data;
  data.push_back(0x7E);
  const uint8_t *ptr = reinterpret_cast<const uint8_t*>(&msg);
  size_t unstuffed_size = sizeof(msg);
  for (size_t i = 0; i < unstuffed_size; i++)
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
std::unique_ptr<T> HandSerial::PPP_unstuff(const std::vector<uint8_t> &stuffed_data)
{
  auto res = std::make_unique<T>();
  uint8_t *ptr = reinterpret_cast<uint8_t*>(res.get());
  size_t unstuffed_size = sizeof(T);

  if (stuffed_data.size() < 2 || stuffed_data[0] != 0x7E || stuffed_data.back() != 0x7E)
  {
    ROS_ERROR("Invalid PPP frame");
    return nullptr;
  }

  size_t read = 0;
  for (size_t i = 1; i < stuffed_data.size(); i++)
  {
    if (stuffed_data[i] == 0x7E)
    {
      break;
    }
    if (read >= unstuffed_size)
    {
      ROS_ERROR("Invalid PPP frame");
      return nullptr;
    }
    if (stuffed_data[i] == 0x7D)
    {
      i++;
      if (i >= stuffed_data.size())
      {
        ROS_ERROR("Invalid PPP frame");
        return nullptr;
      }
      ptr[read++] = stuffed_data[i] ^ 0x20;
    }
    else
    {
      ptr[read++] = stuffed_data[i];
    }
  }
  if (read != unstuffed_size)
  {
    ROS_ERROR("Invalid PPP frame");
    return nullptr;
  }
  return res;
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

template<typename T>
std::unique_ptr<T> HandSerial::receive(const ros::Duration &timeout)
{
  size_t MSG_SIZE = sizeof(T);
  size_t BUF_SIZE = 2*MSG_SIZE + 2;
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
  auto res = PPP_unstuff<T>(buffer);
  if (!res)
  {
    ROS_ERROR("Failed to unstuff PPP frame");
    return nullptr;
  }
  uint8_t checksum = computeChecksum(*res);
  if (checksum != res->checksum)
  {
    ROS_ERROR_STREAM("Checksum mismatch");
    return nullptr;
  }
  return res;
}

} // namespace psyonic_hand_driver