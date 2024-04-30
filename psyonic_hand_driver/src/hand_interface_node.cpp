#include "psyonic_hand_driver/hand_interface.h"
#include "psyonic_hand_driver/hand_ble.h"

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_manager/controller_manager.h>
#include <psyonic_hand_driver/ChangeControlMode.h>
#include <psyonic_hand_driver/ChangeReplyMode.h>

#include <boost/algorithm/string.hpp>

std::unique_ptr<psyonic_hand_driver::PsyonicHand> hand = nullptr;
std::unique_ptr<controller_manager::ControllerManager> cm = nullptr;

ros::Subscriber voltage_command_sub;

const std::vector<std::string> NO_CONTROLLERS = {};
std::vector<std::string> position_controllers;
std::vector<std::string> velocity_controllers;
std::vector<std::string> effort_controllers;
std::vector<std::string> active_controllers = NO_CONTROLLERS;

bool changeControlMode(psyonic_hand_driver::ChangeControlMode::Request &req, psyonic_hand_driver::ChangeControlMode::Response &res)
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
      hand->setControlMode(psyonic_hand_driver::ControlMode::POSITION);
      break;
    }
    case psyonic_hand_driver::ControlMode::VELOCITY:
    {
      cm->switchController(velocity_controllers, active_controllers, controller_manager_msgs::SwitchController::Request::STRICT);
      active_controllers = velocity_controllers;
      hand->setControlMode(psyonic_hand_driver::ControlMode::VELOCITY);
      break;
    }
    case psyonic_hand_driver::ControlMode::TORQUE:
    {
      cm->switchController(effort_controllers, active_controllers, controller_manager_msgs::SwitchController::Request::STRICT);
      active_controllers = effort_controllers;
      hand->setControlMode(psyonic_hand_driver::ControlMode::TORQUE);
      break;
    }
    case psyonic_hand_driver::ControlMode::VOLTAGE:
    {
      cm->switchController(NO_CONTROLLERS, active_controllers, controller_manager_msgs::SwitchController::Request::STRICT);
      active_controllers = NO_CONTROLLERS;
      hand->setControlMode(psyonic_hand_driver::ControlMode::VOLTAGE);
      break;
    }
    case psyonic_hand_driver::ControlMode::READ_ONLY:
    {
      cm->switchController({}, active_controllers, controller_manager_msgs::SwitchController::Request::STRICT);
      active_controllers.clear();
      hand->setControlMode(psyonic_hand_driver::ControlMode::READ_ONLY);
      break;
    }
    default:
    {
      res.success = false;
    }
  }
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
  
  const std::string device = nhp.param<std::string>("device", "usb-FTDI_TTL232R_FTAM6SIZ-if00-port0");

  hand = std::make_unique<psyonic_hand_driver::PsyonicHand>();
  cm = std::make_unique<controller_manager::ControllerManager>(hand.get(), cm_nh);

  voltage_command_sub = cm_nh.subscribe("hand_voltage_controller/command", 1, voltageCommandCallback);

  ros::ServiceServer change_control_mode_srv = nhp.advertiseService("change_control_mode", changeControlMode);

  if (!hand->connect(device))
  {
    ROS_ERROR("Failed to connect to hand");
    return 1;
  }

  ros::Time last_read_time = ros::Time::now();
  ros::Time last_update_time = ros::Time::now();
  ros::Time last_write_time = ros::Time::now();
  while(ros::ok())
  {
    ros::Time read_time = ros::Time::now();
    hand->read(read_time, read_time - last_read_time);
    last_read_time = read_time;
    ros::Time update_time = ros::Time::now();
    cm->update(update_time, update_time - last_update_time);
    last_update_time = update_time;
    ros::Time write_time = ros::Time::now();
    hand->write(write_time, write_time - last_write_time);
    last_write_time = write_time;
  }

  return 0;
}