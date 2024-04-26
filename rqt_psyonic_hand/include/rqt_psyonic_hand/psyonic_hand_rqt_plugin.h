#pragma once

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>

#include "ui_psyonic_hand_rqt_plugin.h"

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
  void publishJointPositionCommand(ros::Publisher* pub, double value);
  void jointPositionCommandSliderMoved(int value);
  void jointPositionCommandSpinBoxEdited();

private:
  Ui::PsyonicHandRqtPlugin ui;
  QWidget* widget;

  ros::Publisher index_joint_pub;
  ros::Publisher middle_joint_pub;
  ros::Publisher ring_joint_pub;
  ros::Publisher pinky_joint_pub;
  ros::Publisher thumb1_joint_pub;
  ros::Publisher thumb2_joint_pub;

  std::unordered_map<QSlider*, ros::Publisher*> joint_pub_map;
  std::unordered_map<QSlider*, QDoubleSpinBox*> joint_slider_spinbox_map;
  std::unordered_map<QDoubleSpinBox*, QSlider*> joint_spinbox_slider_map;
};

} // namespace rqt_psyonic_hand
