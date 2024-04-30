#include "rqt_psyonic_hand/psyonic_hand_rqt_plugin.h"

#include <pluginlib/class_list_macros.h>

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
    ui.thumb2EffortSlider,
    ui.indexVoltageSlider,
    ui.middleVoltageSlider,
    ui.ringVoltageSlider,
    ui.pinkyVoltageSlider,
    ui.thumb1VoltageSlider,
    ui.thumb2VoltageSlider
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
    ui.thumb2EffortSpinBox,
    ui.indexVoltageSpinBox,
    ui.middleVoltageSpinBox,
    ui.ringVoltageSpinBox,
    ui.pinkyVoltageSpinBox,
    ui.thumb1VoltageSpinBox,
    ui.thumb2VoltageSpinBox
  };

  ui.positionCommandsGroupBox->setEnabled(false);
  ui.velocityCommandsGroupBox->setEnabled(false);
  ui.effortCommandsGroupBox->setEnabled(false);
  ui.voltageCommandsGroupBox->setEnabled(false);

  for (size_t i = 0; i < NUM_CONTROLLER_TYPES; ++i)
  {
    joint_pubs[i] = getNodeHandle().advertise<std_msgs::Float64MultiArray>("/hand/hand_" + CONTROLLER_NAMES[i] + "_controller/command", 1);
    joint_commands[i].data.resize(NUM_HAND_JOINTS);
    pub_msg_map[&joint_pubs[i]] = &joint_commands[i];
    for (size_t j = 0; j < NUM_HAND_JOINTS; ++j)
    {
      QSlider *slider = joint_sliders[i * NUM_HAND_JOINTS + j];
      QDoubleSpinBox *spinbox = joint_spinboxes[i * NUM_HAND_JOINTS + j];
      slider_spinbox_map[slider] = spinbox;
      spinbox_slider_map[spinbox] = slider;
      spinbox_pub_map[spinbox] = &joint_pubs[i];
      slider_spinbox_ratio_map[slider] = SLIDER_SPINBOX_RATIOS[i];
      msg_value_spinbox_ratio_map[spinbox] = MSG_VALUE_SPINBOX_RATIOS[i];
      spinbox_msg_value_map[spinbox] = &joint_commands[i].data[j];
      joint_commands[i].data[j] = spinbox->value() * MSG_VALUE_SPINBOX_RATIOS[i];
    }
  }

  connect(this, &PsyonicHandRqtPlugin::jointStateUpdated, this, &PsyonicHandRqtPlugin::updateJointStateGUI);
  joint_state_sub = getNodeHandle().subscribe("/hand/joint_states", 1, &PsyonicHandRqtPlugin::jointStateCallback, this);

  // Match sliders, spinboxes, and publishers
  for (size_t i = 0; i < NUM_CONTROLLERS; ++i)
  {
    connect(joint_sliders[i], &QSlider::sliderMoved, this, &PsyonicHandRqtPlugin::jointCommandSliderMoved);
    connect(joint_spinboxes[i], &QDoubleSpinBox::editingFinished, this, &PsyonicHandRqtPlugin::jointCommandSpinBoxEdited);
  }

  change_control_mode_client = getNodeHandle().serviceClient<psyonic_hand_driver::ChangeControlMode>("/hand_interface/change_control_mode");

  // connect control mode radio buttons
  connect(ui.controlPositionRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
  connect(ui.controlVelocityRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
  connect(ui.controlTorqueRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
  connect(ui.controlVoltageRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
  connect(ui.controlReadOnlyRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
}

void PsyonicHandRqtPlugin::shutdownPlugin()
{
  // unregister all publishers here
  for (size_t i = 0; i < joint_pubs.size(); ++i)
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

void PsyonicHandRqtPlugin::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_state_msg = *msg;
  emit jointStateUpdated();
}

void PsyonicHandRqtPlugin::jointCommandSliderMoved(int value)
{
  QSlider *slider = qobject_cast<QSlider*>(sender());
  QDoubleSpinBox *spinbox = slider_spinbox_map[slider];
  double *msg_value = spinbox_msg_value_map[spinbox];
  ros::Publisher *pub = spinbox_pub_map[spinbox];

  double dbl_val = static_cast<double>(value) / slider_spinbox_ratio_map[slider];
  spinbox->setValue(dbl_val);
  *msg_value = dbl_val * msg_value_spinbox_ratio_map[spinbox];
  pub->publish(*pub_msg_map[pub]);
}

void PsyonicHandRqtPlugin::jointCommandSpinBoxEdited()
{
  QDoubleSpinBox *spinbox = qobject_cast<QDoubleSpinBox*>(sender());
  QSlider *slider = spinbox_slider_map[spinbox];
  double *msg_value = spinbox_msg_value_map[spinbox];
  ros::Publisher *pub = spinbox_pub_map[spinbox];

  double value = spinbox->value();
  slider->setValue(static_cast<int>(value * slider_spinbox_ratio_map[slider]));
  *msg_value = value * msg_value_spinbox_ratio_map[spinbox];
  pub->publish(*pub_msg_map[pub]);
}

void PsyonicHandRqtPlugin::controlModeChanged()
{
  psyonic_hand_driver::ChangeControlMode srv;
  if (ui.controlPositionRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_driver::ChangeControlMode::Request::POSITION_CONTROL;
    ui.velocityCommandsGroupBox->setEnabled(false);
    ui.effortCommandsGroupBox->setEnabled(false);
    ui.voltageCommandsGroupBox->setEnabled(false);
    ui.positionCommandsGroupBox->setEnabled(true);
    joint_pubs[0].publish(*pub_msg_map[&joint_pubs[0]]);
  }
  else if (ui.controlVelocityRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_driver::ChangeControlMode::Request::VELOCITY_CONTROL;
    ui.positionCommandsGroupBox->setEnabled(false);
    ui.effortCommandsGroupBox->setEnabled(false);
    ui.voltageCommandsGroupBox->setEnabled(false);
    ui.velocityCommandsGroupBox->setEnabled(true);
    joint_pubs[1].publish(*pub_msg_map[&joint_pubs[1]]);
  }
  else if (ui.controlTorqueRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_driver::ChangeControlMode::Request::TORQUE_CONTROL;
    ui.positionCommandsGroupBox->setEnabled(false);
    ui.velocityCommandsGroupBox->setEnabled(false);
    ui.voltageCommandsGroupBox->setEnabled(false);
    ui.effortCommandsGroupBox->setEnabled(true);
    joint_pubs[2].publish(*pub_msg_map[&joint_pubs[2]]);
  }
  else if (ui.controlVoltageRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_driver::ChangeControlMode::Request::VOLTAGE_CONTROL;
    ui.positionCommandsGroupBox->setEnabled(false);
    ui.velocityCommandsGroupBox->setEnabled(false);
    ui.effortCommandsGroupBox->setEnabled(false);
    ui.voltageCommandsGroupBox->setEnabled(true);
  }
  else if (ui.controlReadOnlyRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_driver::ChangeControlMode::Request::READ_ONLY;
    ui.positionCommandsGroupBox->setEnabled(false);
    ui.velocityCommandsGroupBox->setEnabled(false);
    ui.effortCommandsGroupBox->setEnabled(false);
    ui.voltageCommandsGroupBox->setEnabled(false);
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

void PsyonicHandRqtPlugin::updateJointStateGUI()
{
  ui.statePosIndexSpinBox->setValue(joint_state_msg.position[0] / MSG_VALUE_SPINBOX_RATIOS[0]);
  ui.statePosMiddleSpinBox->setValue(joint_state_msg.position[1] / MSG_VALUE_SPINBOX_RATIOS[0]);
  ui.statePosRingSpinBox->setValue(joint_state_msg.position[2] / MSG_VALUE_SPINBOX_RATIOS[0]);
  ui.statePosPinkySpinBox->setValue(joint_state_msg.position[3] / MSG_VALUE_SPINBOX_RATIOS[0]);
  ui.statePosThumb1SpinBox->setValue(joint_state_msg.position[4] / MSG_VALUE_SPINBOX_RATIOS[0]);
  ui.statePosThumb2SpinBox->setValue(joint_state_msg.position[5] / MSG_VALUE_SPINBOX_RATIOS[0]);
  ui.stateVelIndexSpinBox->setValue(joint_state_msg.velocity[0] / MSG_VALUE_SPINBOX_RATIOS[1]);
  ui.stateVelMiddleSpinBox->setValue(joint_state_msg.velocity[1] / MSG_VALUE_SPINBOX_RATIOS[1]);
  ui.stateVelRingSpinBox->setValue(joint_state_msg.velocity[2] / MSG_VALUE_SPINBOX_RATIOS[1]);
  ui.stateVelPinkySpinBox->setValue(joint_state_msg.velocity[3] / MSG_VALUE_SPINBOX_RATIOS[1]);
  ui.stateVelThumb1SpinBox->setValue(joint_state_msg.velocity[4] / MSG_VALUE_SPINBOX_RATIOS[1]);
  ui.stateVelThumb2SpinBox->setValue(joint_state_msg.velocity[5] / MSG_VALUE_SPINBOX_RATIOS[1]);
  ui.stateEffIndexSpinBox->setValue(joint_state_msg.effort[0] / MSG_VALUE_SPINBOX_RATIOS[2]);
  ui.stateEffMiddleSpinBox->setValue(joint_state_msg.effort[1] / MSG_VALUE_SPINBOX_RATIOS[2]);
  ui.stateEffRingSpinBox->setValue(joint_state_msg.effort[2] / MSG_VALUE_SPINBOX_RATIOS[2]);
  ui.stateEffPinkySpinBox->setValue(joint_state_msg.effort[3] / MSG_VALUE_SPINBOX_RATIOS[2]);
  ui.stateEffThumb1SpinBox->setValue(joint_state_msg.effort[4] / MSG_VALUE_SPINBOX_RATIOS[2]);
  ui.stateEffThumb2SpinBox->setValue(joint_state_msg.effort[5] / MSG_VALUE_SPINBOX_RATIOS[2]);
}

} // namespace rqt_psyonic_hand

PLUGINLIB_EXPORT_CLASS(rqt_psyonic_hand::PsyonicHandRqtPlugin, rqt_gui_cpp::Plugin)
