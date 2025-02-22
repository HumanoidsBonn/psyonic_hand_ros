#pragma once

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <psyonic_hand_msgs/ControlInterface.h>
#include <psyonic_hand_msgs/ControlMode.h>
#include <psyonic_hand_msgs/ReplyMode.h>
#include <psyonic_hand_msgs/HandStatus.h>
#include <psyonic_hand_msgs/TouchSensorData.h>

#include "ui_psyonic_hand_rqt_plugin.h"

#include <array>

namespace rqt_psyonic_hand
{

class PsyonicHandRqtPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  PsyonicHandRqtPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private slots:
  // UI slots
  void jointCommandSliderMoved(int value);
  void jointCommandSpinBoxEdited();
  void controlInterfaceChanged();
  void controlModeChanged();
  void replyModeChanged(); 
 
  void updateJointStateGUI();
  void updateControlInterfaceGUI();
  void updateControlModeGUI();
  void updateReplyModeGUI();
  void updateHandStatusGUI();
  void updateTouchSensorDataGUI();
  void updateJointCommandGUI();

signals:
  void jointStateUpdated();
  void controlInterfaceUpdated();
  void controlModeUpdated();
  void replyModeUpdated();
  void handStatusUpdated();
  void touchSensorDataUpdated();

private:
  Ui::PsyonicHandRqtPlugin ui;
  QWidget* widget;

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void controlInterfaceCallback(const psyonic_hand_msgs::ControlInterface::ConstPtr& msg);
  void controlModeCallback(const psyonic_hand_msgs::ControlMode::ConstPtr& msg);
  void replyModeCallback(const psyonic_hand_msgs::ReplyMode::ConstPtr& msg);
  void handStatusCallback(const psyonic_hand_msgs::HandStatus::ConstPtr& msg);
  void touchSensorDataCallback(const psyonic_hand_msgs::TouchSensorData::ConstPtr& msg);

  static constexpr size_t NUM_CONTROLLER_TYPES = 4;
  static constexpr size_t NUM_HAND_JOINTS = 6;
  static constexpr size_t NUM_CONTROLLERS = NUM_CONTROLLER_TYPES * NUM_HAND_JOINTS;
  static constexpr size_t NUM_TOUCH_SENSORS = 30;

  bool first_touch_data = true;
  std::array<double, NUM_TOUCH_SENSORS> touch_sensor_avg;

  std::array<std::string, NUM_HAND_JOINTS> JOINT_NAMES = {
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb1",
    "thumb2"
  };

  std::array<std::string, NUM_CONTROLLER_TYPES> CONTROLLER_NAMES = {
    "position",
    "velocity",
    "effort",
    "voltage"
  };

  static constexpr std::array<double, NUM_CONTROLLER_TYPES> SLIDER_SPINBOX_RATIOS = {
    1.0,
    1.0,
    10.0,
    1.0
  };

  static constexpr std::array<double, NUM_CONTROLLER_TYPES> MSG_VALUE_SPINBOX_RATIOS = {
    M_PI / 180.0,
    M_PI / 180.0,
    0.001,
    0.01
  };

  std::array<ros::Publisher, NUM_CONTROLLER_TYPES> joint_pubs;
  std::array<QSlider*, NUM_CONTROLLERS> joint_sliders;
  std::array<QDoubleSpinBox*, NUM_CONTROLLERS> joint_spinboxes;
  std::array<QDoubleSpinBox*, NUM_TOUCH_SENSORS> touch_sensor_spinboxes;
  std::array<std_msgs::Float64MultiArray, NUM_CONTROLLERS> joint_commands;

  ros::Subscriber joint_state_sub;
  sensor_msgs::JointState joint_state_msg;
  ros::Subscriber control_interface_sub;
  psyonic_hand_msgs::ControlInterface control_interface_msg;
  ros::Subscriber control_mode_sub;
  psyonic_hand_msgs::ControlMode control_mode_msg;
  ros::Subscriber reply_mode_sub;
  psyonic_hand_msgs::ReplyMode reply_mode_msg;
  ros::Subscriber hand_status_sub;
  psyonic_hand_msgs::HandStatus hand_status_msg;
  ros::Subscriber touch_sensor_data_sub;
  psyonic_hand_msgs::TouchSensorData touch_sensor_data_msg;

  ros::ServiceClient change_control_interface_client;
  ros::ServiceClient change_control_mode_client;
  ros::ServiceClient change_reply_mode_client;

  std::unordered_map<QSlider*, QDoubleSpinBox*> slider_spinbox_map;
  std::unordered_map<QDoubleSpinBox*, QSlider*> spinbox_slider_map;
  std::unordered_map<QDoubleSpinBox*, ros::Publisher*> spinbox_pub_map;
  std::unordered_map<QDoubleSpinBox*, double> msg_value_spinbox_ratio_map;
  std::unordered_map<QDoubleSpinBox*, double*> spinbox_msg_value_map;
  std::unordered_map<QSlider*, double> slider_spinbox_ratio_map;
  std::unordered_map<ros::Publisher*, std_msgs::Float64MultiArray*> pub_msg_map;
};

} // namespace rqt_psyonic_hand
