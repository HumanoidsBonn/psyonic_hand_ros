#pragma once

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>

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
  void publishJointCommand(ros::Publisher* pub, double value);
  void jointCommandSliderMoved(int value);
  void jointCommandSpinBoxEdited();
  void controlModeChanged();

private:
  Ui::PsyonicHandRqtPlugin ui;
  QWidget* widget;

  static constexpr size_t NUM_CONTROLLERS_PER_JOINT = 3;
  static constexpr size_t NUM_HAND_JOINTS = 6;
  static constexpr size_t NUM_CONTROLLERS = NUM_CONTROLLERS_PER_JOINT * NUM_HAND_JOINTS;

  std::array<std::string, NUM_HAND_JOINTS> JOINT_NAMES = {
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb1",
    "thumb2"
  };

  std::array<std::string, NUM_CONTROLLERS_PER_JOINT> CONTROLLER_NAMES = {
    "position",
    "velocity",
    "effort"
  };

  std::array<ros::Publisher, NUM_CONTROLLERS> joint_pubs;
  std::array<QSlider*, NUM_CONTROLLERS> joint_sliders;
  std::array<QDoubleSpinBox*, NUM_CONTROLLERS> joint_spinboxes;

  ros::ServiceClient change_control_mode_client;
  ros::ServiceClient change_reply_mode_client;

  std::unordered_map<QSlider*, ros::Publisher*> joint_pub_map;
  std::unordered_map<QSlider*, QDoubleSpinBox*> joint_slider_spinbox_map;
  std::unordered_map<QDoubleSpinBox*, QSlider*> joint_spinbox_slider_map;
};

} // namespace rqt_psyonic_hand
