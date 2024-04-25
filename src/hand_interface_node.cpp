#include "psyonic_hand_driver/hand_interface.h"
#include "psyonic_hand_driver/hand_ble.h"
#include "psyonic_hand_driver/HandInterfaceNodeConfig.h"

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <dynamic_reconfigure/server.h>

#include <std_msgs/Float64.h>

std::unique_ptr<psyonic_hand_driver::PsyonicHand> hand = nullptr;
dynamic_reconfigure::Server<psyonic_hand_driver::HandInterfaceNodeConfig> *config_server;
boost::recursive_mutex config_mutex;
psyonic_hand_driver::HandInterfaceNodeConfig current_config;

ros::Publisher index_joint_pub;
ros::Publisher middle_joint_pub;
ros::Publisher ring_joint_pub;
ros::Publisher pinky_joint_pub;
ros::Publisher thumb1_joint_pub;
ros::Publisher thumb2_joint_pub;

/*
config_mutex.lock();
(change stuff in current_config)
config_server->updateConfig(current_config);
config_mutex.unlock();
*/

std_msgs::Float64 toMsg(double value)
{
  std_msgs::Float64 msg;
  msg.data = value;
  return msg;
}

void reconfigureCallback(psyonic_hand_driver::HandInterfaceNodeConfig &config, uint32_t level)
{
  if (!hand)
  {
    ROS_ERROR("Hand not initialized");
    return;
  }
  hand->control_mode = static_cast<psyonic_hand_driver::ControlMode>(config.control_mode);
  hand->reply_mode_request = static_cast<psyonic_hand_driver::ReplyMode>(config.reply_mode);

  if (level & 1 << 2)
  {
    ROS_WARN("Publish joint commands");
    index_joint_pub.publish(toMsg(config.pos_index));
    middle_joint_pub.publish(toMsg(config.pos_middle));
    ring_joint_pub.publish(toMsg(config.pos_ring));
    pinky_joint_pub.publish(toMsg(config.pos_pinky));
    thumb1_joint_pub.publish(toMsg(config.pos_thumb1));
    thumb2_joint_pub.publish(toMsg(config.pos_thumb2));
  }

  current_config = config;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_interface_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

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
  controller_manager::ControllerManager cm(hand.get(), nhp);

  if (!hand->connect(device))
  {
    ROS_ERROR("Failed to connect to hand");
    return 1;
  }

  index_joint_pub = nhp.advertise<std_msgs::Float64>("index_joint_controller/command", 1);
  middle_joint_pub = nhp.advertise<std_msgs::Float64>("middle_joint_controller/command", 1);
  ring_joint_pub = nhp.advertise<std_msgs::Float64>("ring_joint_controller/command", 1);
  pinky_joint_pub = nhp.advertise<std_msgs::Float64>("pinky_joint_controller/command", 1);
  thumb1_joint_pub = nhp.advertise<std_msgs::Float64>("thumb1_joint_controller/command", 1);
  thumb2_joint_pub = nhp.advertise<std_msgs::Float64>("thumb2_joint_controller/command", 1);

  config_server = new dynamic_reconfigure::Server<psyonic_hand_driver::HandInterfaceNodeConfig>(config_mutex, nhp);
  config_server->setCallback(reconfigureCallback);

  ros::Time last_read_time = ros::Time::now();
  ros::Time last_update_time = ros::Time::now();
  ros::Time last_write_time = ros::Time::now();
  while(ros::ok())
  {
    ros::Time read_time = ros::Time::now();
    hand->read(read_time, read_time - last_read_time);
    last_read_time = read_time;
    ros::Time update_time = ros::Time::now();
    cm.update(update_time, update_time - last_update_time);
    last_update_time = update_time;
    ros::Time write_time = ros::Time::now();
    hand->write(write_time, write_time - last_write_time);
    last_write_time = write_time;
  }

  return 0;
}