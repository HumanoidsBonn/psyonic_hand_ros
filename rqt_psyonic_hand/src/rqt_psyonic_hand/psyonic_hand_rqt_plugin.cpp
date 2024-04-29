#include "rqt_psyonic_hand/psyonic_hand_rqt_plugin.h"

#include <pluginlib/class_list_macros.h>

#include <std_msgs/Float64.h>
#include <psyonic_hand_driver/ChangeControlMode.h>
#include <psyonic_hand_driver/ChangeReplyMode.h>

namespace rqt_psyonic_hand
{

PsyonicHandRqtPlugin::PsyonicHandRqtPlugin() :
  rqt_gui_cpp::Plugin(),
  widget(nullptr)
{
  setObjectName("PsyonicHandRqtPlugin");
}

void PsyonicHandRqtPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui.setupUi(widget);
  // add widget to the user interface
  context.addWidget(widget);

  joint_sliders = {
    ui.indexPositionSlider,
    ui.middlePositionSlider,
    ui.ringPositionSlider,
    ui.pinkyPositionSlider,
    ui.thumb1PositionSlider,
    ui.thumb2PositionSlider,
    ui.indexVelocitySlider,
    ui.middleVelocitySlider,
    ui.ringVelocitySlider,
    ui.pinkyVelocitySlider,
    ui.thumb1VelocitySlider,
    ui.thumb2VelocitySlider,
    ui.indexEffortSlider,
    ui.middleEffortSlider,
    ui.ringEffortSlider,
    ui.pinkyEffortSlider,
    ui.thumb1EffortSlider,
    ui.thumb2EffortSlider
  };

  joint_spinboxes = {
    ui.indexPostionSpinBox,
    ui.middlePostionSpinBox,
    ui.ringPostionSpinBox,
    ui.pinkyPostionSpinBox,
    ui.thumb1PostionSpinBox,
    ui.thumb2PostionSpinBox,
    ui.indexVelocitySpinBox,
    ui.middleVelocitySpinBox,
    ui.ringVelocitySpinBox,
    ui.pinkyVelocitySpinBox,
    ui.thumb1VelocitySpinBox,
    ui.thumb2VelocitySpinBox,
    ui.indexEffortSpinBox,
    ui.middleEffortSpinBox,
    ui.ringEffortSpinBox,
    ui.pinkyEffortSpinBox,
    ui.thumb1EffortSpinBox,
    ui.thumb2EffortSpinBox
  };

  ui.jointPositionCommandGroupBox->setVisible(true);
  ui.jointVelocityCommandGroupBox->setVisible(false);
  ui.jointEffortCommandGroupBox->setVisible(false);

  for (size_t i = 0; i < NUM_CONTROLLERS_PER_JOINT; ++i)
  {
    for (size_t j = 0; j < NUM_HAND_JOINTS; ++j)
    {

      joint_pubs[i * NUM_HAND_JOINTS + j] = getNodeHandle().advertise<std_msgs::Float64>("/hand_controllers/" + JOINT_NAMES[j] + "_" + CONTROLLER_NAMES[i] + "_controller/command", 1);
    }
  }

  // Match sliders, spinboxes, and publishers
  for (size_t i = 0; i < NUM_CONTROLLERS; ++i)
  {
    joint_slider_spinbox_map[joint_sliders[i]] = joint_spinboxes[i];
    joint_spinbox_slider_map[joint_spinboxes[i]] = joint_sliders[i];
    joint_pub_map[joint_sliders[i]] = &joint_pubs[i];
    connect(joint_sliders[i], &QSlider::sliderMoved, this, &PsyonicHandRqtPlugin::jointCommandSliderMoved);
    connect(joint_spinboxes[i], &QDoubleSpinBox::editingFinished, this, &PsyonicHandRqtPlugin::jointCommandSpinBoxEdited);
  }

  change_control_mode_client = getNodeHandle().serviceClient<psyonic_hand_driver::ChangeControlMode>("/hand_interface/change_control_mode");

  // connect control mode radio buttons
  connect(ui.controlPositionRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
  connect(ui.controlVelocityRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
  connect(ui.controlTorqueRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
  connect(ui.controlReadOnlyRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
}

void PsyonicHandRqtPlugin::shutdownPlugin()
{
  // unregister all publishers here
  for (size_t i = 0; i < NUM_CONTROLLERS; ++i)
  {
    joint_pubs[i].shutdown();
  }
}

void PsyonicHandRqtPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void PsyonicHandRqtPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void PsyonicHandRqtPlugin::publishJointCommand(ros::Publisher* pub, double value)
{
  std_msgs::Float64 msg;
  msg.data = value * M_PI / 180.0;
  pub->publish(msg);
}

void PsyonicHandRqtPlugin::jointCommandSliderMoved(int value)
{
  QSlider *slider = qobject_cast<QSlider*>(sender());
  joint_slider_spinbox_map[slider]->setValue(static_cast<double>(value));
  publishJointCommand(joint_pub_map[slider], static_cast<double>(value));
}

void PsyonicHandRqtPlugin::jointCommandSpinBoxEdited()
{
  QDoubleSpinBox *spinbox = qobject_cast<QDoubleSpinBox*>(sender());
  double value = spinbox->value();
  QSlider *slider = joint_spinbox_slider_map[spinbox];
  slider->setValue(static_cast<int>(value));
  publishJointCommand(joint_pub_map[slider], value);
}

void PsyonicHandRqtPlugin::controlModeChanged()
{
  psyonic_hand_driver::ChangeControlMode srv;
  if (ui.controlPositionRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_driver::ChangeControlMode::Request::POSITION_CONTROL;
    ui.jointVelocityCommandGroupBox->setVisible(false);
    ui.jointEffortCommandGroupBox->setVisible(false);
    ui.jointPositionCommandGroupBox->setVisible(true);
  }
  else if (ui.controlVelocityRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_driver::ChangeControlMode::Request::VELOCITY_CONTROL;
    ui.jointPositionCommandGroupBox->setVisible(false);
    ui.jointEffortCommandGroupBox->setVisible(false);
    ui.jointVelocityCommandGroupBox->setVisible(true);
  }
  else if (ui.controlTorqueRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_driver::ChangeControlMode::Request::TORQUE_CONTROL;
    ui.jointPositionCommandGroupBox->setVisible(false);
    ui.jointVelocityCommandGroupBox->setVisible(false);
    ui.jointEffortCommandGroupBox->setVisible(true);
  }
  else if (ui.controlReadOnlyRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_driver::ChangeControlMode::Request::READ_ONLY;
  }
  else
  {
    ROS_ERROR("Unknown control mode");
    return;
  }
  if (!change_control_mode_client.call(srv))
  {
    ROS_ERROR("Failed to call service /hand_interface/change_control_mode");
  }
}

} // namespace rqt_psyonic_hand

PLUGINLIB_EXPORT_CLASS(rqt_psyonic_hand::PsyonicHandRqtPlugin, rqt_gui_cpp::Plugin)
