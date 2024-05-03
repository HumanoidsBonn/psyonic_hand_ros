#include "psyonic_hand_driver/hand_ble.h"
#include "psyonic_hand_driver/hand_data_conversion.h"

#include <ros/ros.h>

namespace psyonic_hand_driver
{

HandBLE::HandBLE() : is_connected(false)
{
}

HandBLE::~HandBLE()
{
  if (adapter.initialized() && adapter.scan_is_active())
    adapter.scan_stop();

  if (hand.initialized() && hand.is_connected())
    hand.disconnect();
}

bool HandBLE::initializeService()
{
  bool hand_service_found = false;
  bool hand_rx_characteristic_found = false;
  bool hand_tx_characteristic_found = false;

  for (auto service : hand.services())
  {
    if (service.uuid() == "6e400001-b5a3-f393-e0a9-e50e24dcca9e")
    {
      hand_service = service;
      hand_service_found = true;
      for (auto characteristic : service.characteristics())
      {
        if (characteristic.uuid() == "6e400002-b5a3-f393-e0a9-e50e24dcca9e")
        {
          hand_rx_characteristic = characteristic;
          hand_rx_characteristic_found = true;
        }
        else if (characteristic.uuid() == "6e400003-b5a3-f393-e0a9-e50e24dcca9e")
        {
          hand_tx_characteristic = characteristic;
          hand_tx_characteristic_found = true;

          if (hand_tx_characteristic.can_notify())
          {
            hand.notify(hand_service.uuid(), hand_tx_characteristic.uuid(), std::bind(&HandBLE::handMessageReceived, this, std::placeholders::_1));
          }
          else
          {
            ROS_ERROR("Hand TX characteristic cannot notify");
          }
        }
      }
    }
  }

  if (!hand_service_found)
  {
    ROS_ERROR("Hand service not found");
    return false;
  }
  if (!hand_rx_characteristic_found)
  {
    ROS_ERROR("Hand RX characteristic not found");
    return false;
  }
  if (!hand_tx_characteristic_found)
  {
    ROS_ERROR("Hand TX characteristic not found");
    return false;
  }
  return true;
}

void HandBLE::peripheralFound(SimpleBLE::Peripheral peripheral)
{
  ROS_INFO_STREAM("Peripheral found: " << peripheral.identifier() << ", address " << peripheral.address());
  if (peripheral.identifier() == "PSYONIC-23ABH238")
  {
    ROS_INFO("Found hand");
    hand = peripheral;
  }
}

bool HandBLE::startScanForHand()
{
  if (!SimpleBLE::Adapter::bluetooth_enabled())
  {
    ROS_ERROR("Bluetooth not enabled");
    return false;
  }

  auto adapters = SimpleBLE::Adapter::get_adapters();
  if (adapters.empty())
  {
    ROS_ERROR("No Bluetooth adapters found");
    return false;
  }

  // Use the first adapter found
  adapter = adapters[0];

  ROS_INFO_STREAM("Using adapter " << adapter.identifier() << ", address " << adapter.address());

  adapter.set_callback_on_scan_found(std::bind(&HandBLE::peripheralFound, this, std::placeholders::_1));

  adapter.scan_start();

  return true;
}

bool HandBLE::stopScanForHand()
{
  if (!adapter.initialized())
  {
    ROS_ERROR("Adapter not initialized");
    return false;
  }
  adapter.scan_stop();
  return true;
}

bool HandBLE::connect()
{
  if (!hand.initialized())
  {
    ROS_ERROR("Hand not initialized");
    return false;
  }
  if (hand.is_connected())
  {
    ROS_ERROR("Hand already connected");
    return false;
  }
  hand.connect();
  if (!initializeService())
  {
    ROS_ERROR("Failed to initialize service");
    hand.disconnect();
    return false;
  }
  is_connected = true;
  return true;

}

bool HandBLE::disconnect()
{
  if (!hand.initialized() || !hand.is_connected())
  {
    ROS_ERROR("Hand not connected");
    return false;
  }
  //hand.unsubscribe(hand_service.uuid(), hand_tx_characteristic.uuid());
  //hand.unsubscribe(hand_service.uuid(), hand_rx_characteristic.uuid());
  hand.disconnect();
  is_connected = false;
  return true;
}

void HandBLE::handMessageReceived(SimpleBLE::ByteArray payload)
{
  std::string message(payload.begin(), payload.end());
  ROS_INFO_STREAM("Received message: " << message);
}

void HandBLE::sendCommand(const std::string& command)
{
  if (!is_connected)
  {
    ROS_ERROR("Hand not connected");
    return;
  }

  hand.write_command(hand_service.uuid(), hand_rx_characteristic.uuid(), SimpleBLE::ByteArray(command.begin(), command.end()));
}

void HandBLE::sendPositionCommand(double index, double middle, double ring, double pinky, double thumb_flexor, double thumb_rotator)
{
  FingerControlCommandBLE command;
  command.index_position = radToPos(index);
  command.index_period = secondsToTime(0.2);
  command.middle_position = radToPos(middle);
  command.middle_period = secondsToTime(0.2);
  command.ring_position = radToPos(ring);
  command.ring_period = secondsToTime(0.2);
  command.pinky_position = radToPos(pinky);
  command.pinky_period = secondsToTime(0.2);
  command.thumb_flexor_position = radToPos(thumb_flexor);
  command.thumb_flexor_period = secondsToTime(0.2);
  command.thumb_rotator_position = radToPos(thumb_rotator);
  command.thumb_rotator_period = secondsToTime(0.2);

  SimpleBLE::ByteArray command_array(reinterpret_cast<uint8_t*>(&command), reinterpret_cast<uint8_t*>(&command) + sizeof(FingerControlCommandBLE));
  hand.write_command(hand_service.uuid(), hand_rx_characteristic.uuid(), command_array);
}

} // namespace psyonic_hand_driver