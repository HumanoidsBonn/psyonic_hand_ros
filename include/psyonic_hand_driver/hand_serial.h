#pragma once

#include <vector>
#include <string>
#include <optional>

namespace psyonic_hand_driver
{

class HandSerial
{
private:
  std::vector<std::string> getSerialPorts();
  std::optional<std::string> getSerialPort(const std::string &id);

public:
  HandSerial();
  ~HandSerial();
};

} // namespace psyonic_hand_driver