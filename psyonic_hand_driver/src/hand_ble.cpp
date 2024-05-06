#include "psyonic_hand_driver/hand_ble.h"
#include "psyonic_hand_driver/hand_data_conversion.h"

#include <ros/ros.h>
#include <boost/algorithm/hex.hpp>

namespace psyonic_hand_driver
{

std::string to_command(PlotModeBLE mode)
{
  switch (mode)
  {
  case PlotModeBLE::DISABLE:
    return "P1";
  case PlotModeBLE::TOUCH_SENSOR:
    return "P0";
  case PlotModeBLE::DIRECT_CONTROL:
    return "P2";
  case PlotModeBLE::DIRECT_CONTROL_DISABLE:
    return "P3";
  case PlotModeBLE::FSR:
    return "P4";
  case PlotModeBLE::FSR_CONSOLE:
    return "P41";
  case PlotModeBLE::FINGER_POSITION:
    return "P5";
  case PlotModeBLE::FINGER_VELOCITY:
    return "P8";
  case PlotModeBLE::IMU:
    return "P9";
  case PlotModeBLE::GRIP_PERCENT:
    return "PB";
  case PlotModeBLE::FINGER_POSITION_GOAL:
    return "PC";
  case PlotModeBLE::FINGER_PROGRESS:
    return "PD";
  case PlotModeBLE::MOTOR_CURRENT:
    return "PE";
  case PlotModeBLE::ENCODER_POSITION:
    return "PF";
  case PlotModeBLE::ENCODER_CONSOLE:
    return "PG";
  case PlotModeBLE::BATTERY_VOLTAGE:
    return "PH0";
  case PlotModeBLE::BATTERY_VOLTAGE_CONSOLE:
    return "PH1";
  case PlotModeBLE::HAND_TEMP_CONSOLE:
    return "PI";
  default:
    ROS_ERROR_STREAM("Unknown plot mode: " << static_cast<int>(mode));
    return "P1";
  }
}

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
  if (peripheral.identifier() == "PSYONIC-23ABH238")
  {
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
  try
  {
    hand.connect();
  }
  catch(const SimpleBLE::Exception::BaseException &e)
  {
    ROS_ERROR_STREAM("Failed to connect to hand: " << e.what());
    return false;
  }
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
  hand.disconnect();
  is_connected = false;
  return true;
}

bool HandBLE::sendCommand(const std::string& command)
{
  if (!is_connected)
  {
    ROS_ERROR("Hand not connected");
    return false;
  }

  try
  {
    hand.write_command(hand_service.uuid(), hand_rx_characteristic.uuid(), SimpleBLE::ByteArray(command.begin(), command.end()));
  }
  catch(const SimpleBLE::Exception::BaseException &e)
  {
    ROS_ERROR_STREAM("Failed to send command: " << e.what());
    return false;
  }
  return true;
}

bool HandBLE::setReceiverCallback(std::function<void(SimpleBLE::ByteArray)> callback)
{
  if (!is_connected)
  {
    ROS_ERROR("Hand not connected");
    return false;
  }

  try
  {
    hand.notify(hand_service.uuid(), hand_tx_characteristic.uuid(), std::move(callback));
  }
  catch(const SimpleBLE::Exception::BaseException &e)
  {
    ROS_ERROR_STREAM("Failed to set receiver callback: " << e.what());
    return false;
  }
  return true;
}

bool HandBLE::sendPositionCommand(double index, double middle, double ring, double pinky, double thumb_flexor, double thumb_rotator)
{
  if (!is_connected)
  {
    ROS_ERROR("Hand not connected");
    return false;
  }

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

  try
  {
    hand.write_command(hand_service.uuid(), hand_rx_characteristic.uuid(), command_array);
    //ROS_INFO_STREAM("Send command: " << boost::algorithm::hex(command_array));
  }
  catch(const SimpleBLE::Exception::BaseException &e)
  {
    ROS_ERROR_STREAM("Failed to send position command: " << e.what());
    return false;
  }
  return true;
}

} // namespace psyonic_hand_driver