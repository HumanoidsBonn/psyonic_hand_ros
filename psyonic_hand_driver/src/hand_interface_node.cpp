#include "psyonic_hand_driver/hand_interface.h"
#include "psyonic_hand_driver/hand_ble.h"

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_manager/controller_manager.h>
#include <psyonic_hand_msgs/ControlInterface.h>
#include <psyonic_hand_msgs/ControlMode.h>
#include <psyonic_hand_msgs/ReplyMode.h>
#include <psyonic_hand_msgs/HandStatus.h>
#include <psyonic_hand_msgs/TouchSensorData.h>
#include <psyonic_hand_msgs/ChangeControlInterface.h>
#include <psyonic_hand_msgs/ChangeControlMode.h>
#include <psyonic_hand_msgs/ChangeReplyMode.h>

#include <boost/algorithm/string.hpp>

std::unique_ptr<psyonic_hand_driver::PsyonicHand> hand = nullptr;
std::unique_ptr<controller_manager::ControllerManager> cm = nullptr;

const std::vector<std::string> NO_CONTROLLERS = {};
std::vector<std::string> position_controllers;
std::vector<std::string> velocity_controllers;
std::vector<std::string> effort_controllers;
std::vector<std::string> active_controllers = NO_CONTROLLERS;

bool changeControlInterface(psyonic_hand_msgs::ChangeControlInterface::Request &req, psyonic_hand_msgs::ChangeControlInterface::Response &res)
{
  auto requested_interface = static_cast<psyonic_hand_driver::ControlInterface>(req.control_interface);
  if (requested_interface == hand->getControlInterface())
  {
    res.success = true;
    return true;
  }
  if (requested_interface == psyonic_hand_driver::ControlInterface::SERIAL)
  {
    res.success = hand->setControlInterface(requested_interface);
  }
  else if (requested_interface == psyonic_hand_driver::ControlInterface::BLE)
  {
    if (hand->getControlMode() != psyonic_hand_driver::ControlMode::POSITION)
    {
      cm->switchController(position_controllers, active_controllers, controller_manager_msgs::SwitchController::Request::STRICT);
      active_controllers = position_controllers;
      hand->setControlMode(psyonic_hand_driver::ControlMode::POSITION);
    }
    res.success = hand->setControlInterface(requested_interface);
  }
  else
  {
    ROS_ERROR_STREAM("Unknown control interface: " << static_cast<int>(requested_interface));
    res.success = false;
  }
  return true;
}

bool changeControlMode(psyonic_hand_msgs::ChangeControlMode::Request &req, psyonic_hand_msgs::ChangeControlMode::Response &res)
{
  auto requested_mode = static_cast<psyonic_hand_driver::ControlMode>(req.control_mode);
  if (requested_mode == hand->getControlMode())
  {
    res.success = true;
    return true;
  }
  switch(requested_mode)
  {
    case psyonic_hand_driver::ControlMode::POSITION:
    {
      cm->switchController(position_controllers, active_controllers, controller_manager_msgs::SwitchController::Request::STRICT);
      active_controllers = position_controllers;
      res.success = hand->setControlMode(psyonic_hand_driver::ControlMode::POSITION);
      break;
    }
    case psyonic_hand_driver::ControlMode::VELOCITY:
    {
      cm->switchController(velocity_controllers, active_controllers, controller_manager_msgs::SwitchController::Request::STRICT);
      active_controllers = velocity_controllers;
      res.success = hand->setControlMode(psyonic_hand_driver::ControlMode::VELOCITY);
      break;
    }
    case psyonic_hand_driver::ControlMode::TORQUE:
    {
      cm->switchController(effort_controllers, active_controllers, controller_manager_msgs::SwitchController::Request::STRICT);
      active_controllers = effort_controllers;
      res.success = hand->setControlMode(psyonic_hand_driver::ControlMode::TORQUE);
      break;
    }
    case psyonic_hand_driver::ControlMode::VOLTAGE:
    {
      cm->switchController(NO_CONTROLLERS, active_controllers, controller_manager_msgs::SwitchController::Request::STRICT);
      active_controllers = NO_CONTROLLERS;
      res.success = hand->setControlMode(psyonic_hand_driver::ControlMode::VOLTAGE);
      break;
    }
    case psyonic_hand_driver::ControlMode::READ_ONLY:
    {
      cm->switchController({}, active_controllers, controller_manager_msgs::SwitchController::Request::STRICT);
      active_controllers.clear();
      res.success = hand->setControlMode(psyonic_hand_driver::ControlMode::READ_ONLY);
      break;
    }
    default:
    {
      ROS_ERROR_STREAM("Unknown control mode: " << static_cast<int>(requested_mode));
      res.success = false;
    }
  }
  return true;
}

bool changeReplyMode(psyonic_hand_msgs::ChangeReplyMode::Request &req, psyonic_hand_msgs::ChangeReplyMode::Response &res)
{
  auto requested_mode = static_cast<psyonic_hand_driver::ReplyMode>(req.reply_mode);
  if (requested_mode == hand->getReplyMode())
  {
    res.success = true;
    return true;
  }
  res.success = hand->setReplyMode(requested_mode);
  return true;
}

void voltageCommandCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  if (msg->data.size() != 6)
  {
    ROS_ERROR("Voltage command must have 6 elements");
    return;
  }
  hand->setVoltageCommand(msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_interface_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  const std::string controller_namespace = nhp.param<std::string>("controller_namespace", "hand");
  const std::string position_controllers_str = nhp.param<std::string>("position_controllers", "hand_position_controller");
  boost::algorithm::split(position_controllers, position_controllers_str, boost::is_any_of(" "));
  const std::string velocity_controllers_str = nhp.param<std::string>("velocity_controllers", "hand_velocity_controller");
  boost::algorithm::split(velocity_controllers, velocity_controllers_str, boost::is_any_of(" "));
  const std::string effort_controllers_str = nhp.param<std::string>("effort_controllers", "hand_effort_controller");
  boost::algorithm::split(effort_controllers, effort_controllers_str, boost::is_any_of(" "));

  ros::NodeHandle cm_nh(controller_namespace);

  /*psyonic_hand_driver::HandBLE hand_ble;
  hand_ble.startScanForHand();

  for (ros::Rate r(50); ros::ok(); r.sleep())
  {
    ROS_INFO_THROTTLE(1, "Scanning for hand...");
    if (hand_ble.isConnected())
    {
      ROS_INFO("Connected to hand");
      break;
    }
  }

  if (!hand_ble.isConnected())
  {
    ROS_ERROR("Failed to connect to hand");
    return 1;
  }

  hand_ble.sendCommand("g03:0.20");

  ros::Duration(5.0).sleep();

  hand_ble.sendCommand("g08:0.20");*/
  
  const double max_frequency = nhp.param<double>("max_frequency", 100.0);
  const bool connect_serial = nhp.param<bool>("connect_serial", true);
  const std::string id = nhp.param<std::string>("id", "");
  const std::string port = nhp.param<std::string>("port", "/dev/ttyUSB0");
  const bool connect_ble = nhp.param<bool>("connect_ble", false);

  if (!connect_ble && !connect_serial)
  {
    ROS_ERROR("Must connect to either serial or BLE, or both");
    return 1;
  }

  hand = std::make_unique<psyonic_hand_driver::PsyonicHand>();
  cm = std::make_unique<controller_manager::ControllerManager>(hand.get(), cm_nh);

  ros::Subscriber voltage_command_sub = cm_nh.subscribe("hand_voltage_controller/command", 1, voltageCommandCallback);

  ros::Publisher control_interface_pub = nhp.advertise<psyonic_hand_msgs::ControlInterface>("control_interface", 1, false);
  ros::Publisher control_mode_pub = nhp.advertise<psyonic_hand_msgs::ControlMode>("control_mode", 1, false);
  ros::Publisher reply_mode_pub = nhp.advertise<psyonic_hand_msgs::ReplyMode>("reply_mode", 1, false);
  ros::Publisher hand_status_pub = nhp.advertise<psyonic_hand_msgs::HandStatus>("hand_status", 1, false);
  ros::Publisher touch_sensor_pub = nhp.advertise<psyonic_hand_msgs::TouchSensorData>("touch_sensor_data", 1, false);

  psyonic_hand_msgs::ControlInterface control_interface_msg;
  control_interface_msg.control_interface = static_cast<uint8_t>(hand->getControlInterface());

  psyonic_hand_msgs::ControlMode control_mode_msg;
  control_mode_msg.control_mode = static_cast<uint8_t>(hand->getControlMode());

  psyonic_hand_msgs::ReplyMode reply_mode_msg;
  reply_mode_msg.reply_mode = static_cast<uint8_t>(hand->getReplyMode());

  psyonic_hand_msgs::HandStatus hand_status_msg;
  
  psyonic_hand_msgs::TouchSensorData touch_sensor_msg;
  static_assert(sizeof(psyonic_hand_msgs::TouchSensorData) == sizeof(psyonic_hand_driver::UnpackedTouchSensorData), "TouchSensorData message size mismatch");

  ros::ServiceServer change_control_interface_srv = nhp.advertiseService("change_control_interface", changeControlInterface);
  ros::ServiceServer change_control_mode_srv = nhp.advertiseService("change_control_mode", changeControlMode);
  ros::ServiceServer change_reply_mode_srv = nhp.advertiseService("change_reply_mode", changeReplyMode);

  if (connect_serial)
  {
    if (id.empty())
    {
      ROS_INFO_STREAM("Connecting to hand on port " << port);
      if (!hand->connectSerial(port))
      {
        ROS_ERROR("Failed to connect to hand");
        return 1;
      }
    }
    else
    {
      ROS_INFO_STREAM("Connecting to hand with ID " << id);
      if (!hand->connectSerialById(id))
      {
        ROS_ERROR("Failed to connect to hand");
        return 1;
      }
    }
    hand_status_msg.serial_connected = true;
  }
  
  if (connect_ble)
  {
    if (!hand->connectBLE(ros::Duration(5.0)))
    {
      ROS_ERROR("Failed to connect bluetooth");
      if (!connect_serial)
      {
        return 1;
      }
    }
    if (!connect_serial)
    {
      hand->setControlInterface(psyonic_hand_driver::ControlInterface::BLE);
    }
    hand_status_msg.bluetooth_connected = true;
  }

  ros::Time last_read_time = ros::Time::now();
  ros::Time last_update_time = ros::Time::now();
  ros::Time last_write_time = ros::Time::now();
  ros::Time loop_freq_measure_time = ros::Time::now();
  size_t loop_count = 0;
  for(ros::Rate r(max_frequency); ros::ok(); r.sleep())
  {
    ros::Time read_time = ros::Time::now();
    hand->read(read_time, read_time - last_read_time);
    last_read_time = read_time;
    ros::Time update_time = ros::Time::now();
    cm->update(update_time, update_time - last_update_time);
    last_update_time = update_time;

    control_interface_msg.control_interface = static_cast<uint8_t>(hand->getControlInterface());
    control_interface_pub.publish(control_interface_msg);
    control_mode_msg.control_mode = static_cast<uint8_t>(hand->getControlMode());
    control_mode_pub.publish(control_mode_msg);
    reply_mode_msg.reply_mode = static_cast<uint8_t>(hand->getReplyMode());
    reply_mode_pub.publish(reply_mode_msg);
    hand_status_pub.publish(hand_status_msg);
  
    psyonic_hand_driver::UnpackedTouchSensorData *touch_sensor_data = hand->getTouchSensorData();
    if (touch_sensor_data)
    {
      memcpy(&touch_sensor_msg, touch_sensor_data, sizeof(psyonic_hand_driver::UnpackedTouchSensorData));
      touch_sensor_pub.publish(touch_sensor_msg);
    }

    ros::Time write_time = ros::Time::now();
    hand->write(write_time, write_time - last_write_time);
    last_write_time = write_time;

    ros::Time cur_loop_time = ros::Time::now();
    loop_count++;
    if (cur_loop_time - loop_freq_measure_time > ros::Duration(1.0))
    {
      ROS_INFO_STREAM("Loop frequency: " << loop_count << " Hz");
      loop_count = 0;
      loop_freq_measure_time = cur_loop_time;
    }
  }

  hand->disconnectBLE();
  ROS_INFO_STREAM("Disconnected from hand");

  return 0;
}