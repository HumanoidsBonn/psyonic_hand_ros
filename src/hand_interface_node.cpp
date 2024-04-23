#include "psyonic_hand_driver/hand_interface.h"
#include "psyonic_hand_driver/hand_ble.h"

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_interface_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  psyonic_hand_driver::HandBLE hand_ble;
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

  hand_ble.sendCommand("g08:0.20");

  ros::waitForShutdown();

  return 0;

  const std::string device = nhp.param<std::string>("device", "usb-FTDI_TTL232R_FTAM6SIZ-if00-port0");

  psyonic_hand_driver::PsyonicHand hand;
  controller_manager::ControllerManager cm(&hand, nhp);

  if (!hand.connect(device))
  {
    ROS_ERROR("Failed to connect to hand");
    return 1;
  }

  for (ros::Rate r(200); ros::ok(); r.sleep())
  {
    auto time = ros::Time::now();
    hand.read(time, r.expectedCycleTime()*2);
    cm.update(time, r.expectedCycleTime()*2);
    r.sleep();
    hand.write(ros::Time::now(), r.expectedCycleTime()*2);
  }

  return 0;
}