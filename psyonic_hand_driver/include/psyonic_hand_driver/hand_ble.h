#pragma once

#include <simpleble/SimpleBLE.h>

namespace psyonic_hand_driver
{

enum class PlotModeBLE
{
  DISABLE,
  TOUCH_SENSOR, // 6x 32-bit float
  DIRECT_CONTROL, // 6x 32-bit float
  DIRECT_CONTROL_DISABLE,
  FSR, // 60x 16-bit int (6x per finger)
  FSR_CONSOLE, // print FSR values to console
  FINGER_POSITION, // 6x 32-bit float
  FINGER_VELOCITY, // 6x 32-bit float
  IMU, // 7x 32-bit float
  GRIP_PERCENT, // 2x 32-bit float
  FINGER_POSITION_GOAL, // 6x 32-bit float
  FINGER_PROGRESS, // 6x 32-bit float
  MOTOR_CURRENT, // 6x 32-bit float
  ENCODER_POSITION, // 1x 32-bit float
  ENCODER_CONSOLE, // print encoder position to console
  BATTERY_VOLTAGE, // 1x 32-bit float
  BATTERY_VOLTAGE_CONSOLE, // print battery voltage to console
  HAND_TEMP_CONSOLE // print motor junction temperatures to console
};

std::string to_command(PlotModeBLE mode);

static constexpr size_t FINGER_CONTROL_COMMAND_BLE_SIZE = 25;

struct __attribute__((__packed__)) FingerControlCommandBLE
{
  uint8_t header = 0x4D;
  int16_t index_position;
  uint16_t index_period;
  int16_t middle_position;
  uint16_t middle_period;
  int16_t ring_position;
  uint16_t ring_period;
  int16_t pinky_position;
  uint16_t pinky_period;
  int16_t thumb_flexor_position;
  uint16_t thumb_flexor_period;
  int16_t thumb_rotator_position;
  uint16_t thumb_rotator_period;
};

static_assert(sizeof(FingerControlCommandBLE) == FINGER_CONTROL_COMMAND_BLE_SIZE, "FingerControlCommandBLE size not correct");

class HandBLE
{
private:
  SimpleBLE::Adapter adapter;
  SimpleBLE::Peripheral hand;
  SimpleBLE::Service hand_service;
  SimpleBLE::Characteristic hand_rx_characteristic;
  SimpleBLE::Characteristic hand_tx_characteristic;

  volatile bool is_connected;

  void peripheralFound(SimpleBLE::Peripheral peripheral);

  bool initializeService();

public:
  HandBLE();
  ~HandBLE();

  bool startScanForHand();
  bool stopScanForHand();
  bool deviceFound() { return hand.initialized(); }
  bool isConnected() { return is_connected; }
  bool connect();
  bool disconnect();

  bool sendCommand(const std::string& command);

  bool setReceiverCallback(std::function<void(SimpleBLE::ByteArray)> callback);

  bool sendPositionCommand(double index, double middle, double ring, double pinky, double thumb_flexor, double thumb_rotator);
};

} // namespace psyonic_hand_driver