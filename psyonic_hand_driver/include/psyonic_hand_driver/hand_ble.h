#pragma once

#include <simpleble/SimpleBLE.h>

namespace psyonic_hand_driver
{

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
  void handMessageReceived(SimpleBLE::ByteArray payload);

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

  void sendCommand(const std::string& command);

  void sendPositionCommand(double index, double middle, double ring, double pinky, double thumb_flexor, double thumb_rotator);
};

} // namespace psyonic_hand_driver