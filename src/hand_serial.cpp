#include "psyonic_hand_driver/hand_serial.h"

#include <filesystem>

#include <ros/ros.h>

namespace psyonic_hand_driver
{

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
std::unique_ptr<TouchSensorData> HandReplyV1or2::unpackTouchSensorData() const
{
  static constexpr size_t NUM_VALS = sizeof(TouchSensorData) / 2;
  static_assert(NUM_VALS == sizeof(touch_sensor_data) * 2 / 3, "Packed and unpacked touch sensor data sizes do not match");
  auto unpacked_touch_data = std::make_unique<TouchSensorData>();
  uint16_t *vals = reinterpret_cast<uint16_t*>(unpacked_touch_data.get());
  for(int bidx = NUM_VALS * 12 - 4; bidx >= 0; bidx -= 4)
  {
    int validx = bidx / 12;
    int arridx = bidx / 8;
    int shift_val = (bidx % 8);
    vals[validx] |= ((touch_sensor_data[arridx] >> shift_val) &0x0F) << (bidx % 12);
  }
  return unpacked_touch_data;
}
#pragma GCC diagnostic pop

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
    sp.setTimeout(serial::Timeout::max(), 100, 0, 100, 0);
    sp.open();
  }
  catch (const serial::IOException &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }
  return true;
}

} // namespace psyonic_hand_driver