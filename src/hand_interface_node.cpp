#include "psyonic_hand_driver/hand_interface.h"

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_interface_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  const std::string device = nhp.param<std::string>("device", "usb-FTDI_USB-RS422_Cable_FT75P0SM-if00-port0");

  psyonic_hand_driver::PsyonicHand hand;
  controller_manager::ControllerManager cm(&hand, nhp);

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