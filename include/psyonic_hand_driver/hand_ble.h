#pragma once

#include <simpleble/SimpleBLE.h>

namespace psyonic_hand_driver
{

class HandBLE
{
private:
  SimpleBLE::Adapter adapter;
  SimpleBLE::Peripheral hand;
  SimpleBLE::Service hand_service;
  SimpleBLE::Characteristic hand_rx_characteristic;
  SimpleBLE::Characteristic hand_tx_characteristic;

  bool is_connected;

  void peripheralFound(SimpleBLE::Peripheral peripheral);
  void handMessageReceived(SimpleBLE::ByteArray payload);

public:
  HandBLE();
  ~HandBLE();

  bool startScanForHand();
  bool isConnected() { return is_connected; }

  void sendCommand(const std::string& command);
};

} // namespace psyonic_hand_driver