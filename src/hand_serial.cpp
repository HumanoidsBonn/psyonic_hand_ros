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

} // namespace psyonic_hand_driver