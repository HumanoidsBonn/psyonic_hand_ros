#include "rqt_psyonic_hand/psyonic_hand_rqt_plugin.h"

#include <pluginlib/class_list_macros.h>

#include <psyonic_hand_msgs/ChangeControlInterface.h>
#include <psyonic_hand_msgs/ChangeControlMode.h>
#include <psyonic_hand_msgs/ChangeReplyMode.h>

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

  touch_sensor_spinboxes = {
    ui.touchIndex0SpinBox,
    ui.touchIndex1SpinBox,
    ui.touchIndex2SpinBox,
    ui.touchIndex3SpinBox,
    ui.touchIndex4SpinBox,
    ui.touchIndex5SpinBox,
    ui.touchMiddle0SpinBox,
    ui.touchMiddle1SpinBox,
    ui.touchMiddle2SpinBox,
    ui.touchMiddle3SpinBox,
    ui.touchMiddle4SpinBox,
    ui.touchMiddle5SpinBox,
    ui.touchRing0SpinBox,
    ui.touchRing1SpinBox,
    ui.touchRing2SpinBox,
    ui.touchRing3SpinBox,
    ui.touchRing4SpinBox,
    ui.touchRing5SpinBox,
    ui.touchPinky0SpinBox,
    ui.touchPinky1SpinBox,
    ui.touchPinky2SpinBox,
    ui.touchPinky3SpinBox,
    ui.touchPinky4SpinBox,
    ui.touchPinky5SpinBox,
    ui.touchThumb0SpinBox,
    ui.touchThumb1SpinBox,
    ui.touchThumb2SpinBox,
    ui.touchThumb3SpinBox,
    ui.touchThumb4SpinBox,
    ui.touchThumb5SpinBox
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
  connect(this, &PsyonicHandRqtPlugin::controlInterfaceUpdated, this, &PsyonicHandRqtPlugin::updateControlInterfaceGUI);
  connect(this, &PsyonicHandRqtPlugin::controlModeUpdated, this, &PsyonicHandRqtPlugin::updateControlModeGUI);
  connect(this, &PsyonicHandRqtPlugin::replyModeUpdated, this, &PsyonicHandRqtPlugin::updateReplyModeGUI);
  connect(this, &PsyonicHandRqtPlugin::handStatusUpdated, this, &PsyonicHandRqtPlugin::updateHandStatusGUI);
  connect(this, &PsyonicHandRqtPlugin::touchSensorDataUpdated, this, &PsyonicHandRqtPlugin::updateTouchSensorDataGUI);
  joint_state_sub = getNodeHandle().subscribe("/hand/joint_states", 1, &PsyonicHandRqtPlugin::jointStateCallback, this);
  control_interface_sub = getNodeHandle().subscribe("/hand_interface/control_interface", 1, &PsyonicHandRqtPlugin::controlInterfaceCallback, this);
  control_mode_sub = getNodeHandle().subscribe("/hand_interface/control_mode", 1, &PsyonicHandRqtPlugin::controlModeCallback, this);
  reply_mode_sub = getNodeHandle().subscribe("/hand_interface/reply_mode", 1, &PsyonicHandRqtPlugin::replyModeCallback, this);
  hand_status_sub = getNodeHandle().subscribe("/hand_interface/hand_status", 1, &PsyonicHandRqtPlugin::handStatusCallback, this);
  touch_sensor_data_sub = getNodeHandle().subscribe("/hand_interface/touch_sensor_data", 1, &PsyonicHandRqtPlugin::touchSensorDataCallback, this);

  // Match sliders, spinboxes, and publishers
  for (size_t i = 0; i < NUM_CONTROLLERS; ++i)
  {
    connect(joint_sliders[i], &QSlider::sliderMoved, this, &PsyonicHandRqtPlugin::jointCommandSliderMoved);
    connect(joint_spinboxes[i], &QDoubleSpinBox::editingFinished, this, &PsyonicHandRqtPlugin::jointCommandSpinBoxEdited);
  }

  change_control_interface_client = getNodeHandle().serviceClient<psyonic_hand_msgs::ChangeControlInterface>("/hand_interface/change_control_interface");
  change_control_mode_client = getNodeHandle().serviceClient<psyonic_hand_msgs::ChangeControlMode>("/hand_interface/change_control_mode");
  change_reply_mode_client = getNodeHandle().serviceClient<psyonic_hand_msgs::ChangeReplyMode>("/hand_interface/change_reply_mode");

  // connect control mode radio buttons
  connect(ui.interfaceSerialRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlInterfaceChanged);
  connect(ui.interfaceBleRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlInterfaceChanged);
  connect(ui.controlPositionRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
  connect(ui.controlVelocityRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
  connect(ui.controlTorqueRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
  connect(ui.controlVoltageRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
  connect(ui.controlReadOnlyRadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::controlModeChanged);
  connect(ui.replyV1RadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::replyModeChanged);
  connect(ui.replyV2RadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::replyModeChanged);
  connect(ui.replyV3RadioButton, &QRadioButton::released, this, &PsyonicHandRqtPlugin::replyModeChanged);
}

void PsyonicHandRqtPlugin::shutdownPlugin()
{
  // unregister all publishers here
  for (size_t i = 0; i < joint_pubs.size(); ++i)
  {
    joint_pubs[i].shutdown();
  }
  joint_state_sub.shutdown();
  control_interface_sub.shutdown();
  control_mode_sub.shutdown();
  reply_mode_sub.shutdown();
  hand_status_sub.shutdown();
  touch_sensor_data_sub.shutdown();
  change_control_interface_client.shutdown();
  change_control_mode_client.shutdown();
  change_reply_mode_client.shutdown();
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

void PsyonicHandRqtPlugin::controlInterfaceCallback(const psyonic_hand_msgs::ControlInterface::ConstPtr& msg)
{
  control_interface_msg = *msg;
  emit controlInterfaceUpdated();
}

void PsyonicHandRqtPlugin::controlModeCallback(const psyonic_hand_msgs::ControlMode::ConstPtr& msg)
{
  control_mode_msg = *msg;
  emit controlModeUpdated();
}

void PsyonicHandRqtPlugin::replyModeCallback(const psyonic_hand_msgs::ReplyMode::ConstPtr& msg)
{
  reply_mode_msg = *msg;
  emit replyModeUpdated();
}

void PsyonicHandRqtPlugin::handStatusCallback(const psyonic_hand_msgs::HandStatus::ConstPtr& msg)
{
  hand_status_msg = *msg;
  emit handStatusUpdated();
}

void PsyonicHandRqtPlugin::touchSensorDataCallback(const psyonic_hand_msgs::TouchSensorData::ConstPtr& msg)
{
  touch_sensor_data_msg = *msg;
  emit touchSensorDataUpdated();
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

void PsyonicHandRqtPlugin::controlInterfaceChanged()
{
  psyonic_hand_msgs::ChangeControlInterface srv;
  if (ui.interfaceSerialRadioButton->isChecked())
  {
    srv.request.control_interface = psyonic_hand_msgs::ChangeControlInterface::Request::SERIAL;
  }
  else if (ui.interfaceBleRadioButton->isChecked())
  {
    srv.request.control_interface = psyonic_hand_msgs::ChangeControlInterface::Request::BLE;
  }
  else
  {
    ROS_ERROR("Unknown control interface");
    return;
  }
  if (!change_control_interface_client.call(srv))
  {
    ROS_ERROR("Failed to call service /hand_interface/change_control_interface");
    return;
  }
  if (!srv.response.success)
  {
    ROS_ERROR("Failed to change control interface");
    return;
  }
}

void PsyonicHandRqtPlugin::controlModeChanged()
{
  psyonic_hand_msgs::ChangeControlMode srv;
  if (ui.controlPositionRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_msgs::ChangeControlMode::Request::POSITION_CONTROL;
    joint_pubs[0].publish(*pub_msg_map[&joint_pubs[0]]);
  }
  else if (ui.controlVelocityRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_msgs::ChangeControlMode::Request::VELOCITY_CONTROL;
    joint_pubs[1].publish(*pub_msg_map[&joint_pubs[1]]);
  }
  else if (ui.controlTorqueRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_msgs::ChangeControlMode::Request::TORQUE_CONTROL;
    joint_pubs[2].publish(*pub_msg_map[&joint_pubs[2]]);
  }
  else if (ui.controlVoltageRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_msgs::ChangeControlMode::Request::VOLTAGE_CONTROL;
  }
  else if (ui.controlReadOnlyRadioButton->isChecked())
  {
    srv.request.control_mode = psyonic_hand_msgs::ChangeControlMode::Request::READ_ONLY;
  }
  else
  {
    ROS_ERROR("Unknown control mode");
    return;
  }
  if (!change_control_mode_client.call(srv))
  {
    ROS_ERROR("Failed to call service /hand_interface/change_control_mode");
    return;
  }
  if (!srv.response.success)
  {
    ROS_ERROR("Failed to change control mode");
    return;
  }
  updateJointCommandGUI();
}

void PsyonicHandRqtPlugin::replyModeChanged()
{
  psyonic_hand_msgs::ChangeReplyMode srv;
  if (ui.replyV1RadioButton->isChecked())
  {
    srv.request.reply_mode = psyonic_hand_msgs::ChangeReplyMode::Request::REPLY_V1;
  }
  else if (ui.replyV2RadioButton->isChecked())
  {
    srv.request.reply_mode = psyonic_hand_msgs::ChangeReplyMode::Request::REPLY_V2;
  }
  else if (ui.replyV3RadioButton->isChecked())
  {
    srv.request.reply_mode = psyonic_hand_msgs::ChangeReplyMode::Request::REPLY_V3;
  }
  else
  {
    ROS_ERROR("Unknown reply mode");
    return;
  }
  if (!change_reply_mode_client.call(srv))
  {
    ROS_ERROR("Failed to call service /hand_interface/change_reply_mode");
    return;
  }
  if (!srv.response.success)
  {
    ROS_ERROR("Failed to change reply mode");
    return;
  }
}

void PsyonicHandRqtPlugin::updateHandStatusGUI()
{
  ui.interfaceSerialRadioButton->setEnabled(hand_status_msg.serial_connected);
  ui.interfaceBleRadioButton->setEnabled(hand_status_msg.bluetooth_connected);
}

void PsyonicHandRqtPlugin::updateTouchSensorDataGUI()
{
  constexpr double C1 = 121591.0;
  constexpr double C2 = 0.878894;
  const double alpha = ui.touchSmoothingSpinBox->value(); // smoothing factor
  const double touch_sensor_threshold = ui.touchThresholdSpinBox->value();
  uint16_t *data = reinterpret_cast<uint16_t*>(&touch_sensor_data_msg);
  for (size_t i = 0; i < NUM_TOUCH_SENSORS; ++i)
  {
    double V = static_cast<double>(data[i]) * 3.3 / 4096.0;
    double R = 33000.0 / V + 10000.0;
    if (first_touch_data)
      touch_sensor_avg[i] = C1 / R + C2;
    else
      touch_sensor_avg[i] = (1 - alpha) * touch_sensor_avg[i] + alpha * (C1 / R + C2);

    touch_sensor_spinboxes[i]->setValue(touch_sensor_avg[i]); // Values between ~0.88 and 6.96

    if (touch_sensor_avg[i] > touch_sensor_threshold)
    {
      touch_sensor_spinboxes[i]->setStyleSheet("QDoubleSpinBox { background-color: red; }");
    }
    else
    {
      touch_sensor_spinboxes[i]->setStyleSheet("");
    }
  }
  first_touch_data = false;
}

void PsyonicHandRqtPlugin::updateJointStateGUI()
{
  ui.statePosIndexSpinBox->setValue(joint_state_msg.position[0] / MSG_VALUE_SPINBOX_RATIOS[0]);
  ui.statePosMiddleSpinBox->setValue(joint_state_msg.position[1] / MSG_VALUE_SPINBOX_RATIOS[0]);
  ui.statePosRingSpinBox->setValue(joint_state_msg.position[3] / MSG_VALUE_SPINBOX_RATIOS[0]);
  ui.statePosPinkySpinBox->setValue(joint_state_msg.position[2] / MSG_VALUE_SPINBOX_RATIOS[0]);
  ui.statePosThumb1SpinBox->setValue(joint_state_msg.position[4] / MSG_VALUE_SPINBOX_RATIOS[0]);
  ui.statePosThumb2SpinBox->setValue(joint_state_msg.position[5] / MSG_VALUE_SPINBOX_RATIOS[0]);
  ui.stateVelIndexSpinBox->setValue(joint_state_msg.velocity[0] / MSG_VALUE_SPINBOX_RATIOS[1]);
  ui.stateVelMiddleSpinBox->setValue(joint_state_msg.velocity[1] / MSG_VALUE_SPINBOX_RATIOS[1]);
  ui.stateVelRingSpinBox->setValue(joint_state_msg.velocity[3] / MSG_VALUE_SPINBOX_RATIOS[1]);
  ui.stateVelPinkySpinBox->setValue(joint_state_msg.velocity[2] / MSG_VALUE_SPINBOX_RATIOS[1]);
  ui.stateVelThumb1SpinBox->setValue(joint_state_msg.velocity[4] / MSG_VALUE_SPINBOX_RATIOS[1]);
  ui.stateVelThumb2SpinBox->setValue(joint_state_msg.velocity[5] / MSG_VALUE_SPINBOX_RATIOS[1]);
  ui.stateEffIndexSpinBox->setValue(joint_state_msg.effort[0] / MSG_VALUE_SPINBOX_RATIOS[2]);
  ui.stateEffMiddleSpinBox->setValue(joint_state_msg.effort[1] / MSG_VALUE_SPINBOX_RATIOS[2]);
  ui.stateEffRingSpinBox->setValue(joint_state_msg.effort[3] / MSG_VALUE_SPINBOX_RATIOS[2]);
  ui.stateEffPinkySpinBox->setValue(joint_state_msg.effort[2] / MSG_VALUE_SPINBOX_RATIOS[2]);
  ui.stateEffThumb1SpinBox->setValue(joint_state_msg.effort[4] / MSG_VALUE_SPINBOX_RATIOS[2]);
  ui.stateEffThumb2SpinBox->setValue(joint_state_msg.effort[5] / MSG_VALUE_SPINBOX_RATIOS[2]);
}

void PsyonicHandRqtPlugin::updateControlInterfaceGUI()
{
  switch (control_interface_msg.control_interface)
  {
    case psyonic_hand_msgs::ControlInterface::SERIAL:
      ui.interfaceSerialRadioButton->setChecked(true);
      ui.controlVelocityRadioButton->setEnabled(true);
      ui.controlTorqueRadioButton->setEnabled(true);
      ui.controlVoltageRadioButton->setEnabled(true);
      ui.replyModeGroupBox->setEnabled(true);
      break;
    case psyonic_hand_msgs::ControlInterface::BLE:
      ui.interfaceBleRadioButton->setChecked(true);
      ui.controlVelocityRadioButton->setEnabled(false);
      ui.controlTorqueRadioButton->setEnabled(false);
      ui.controlVoltageRadioButton->setEnabled(false);
      ui.replyModeGroupBox->setEnabled(false);
      break;
    default:
      ROS_ERROR("Unknown control interface");
      return;
  }
}

void PsyonicHandRqtPlugin::updateControlModeGUI()
{
  switch (control_mode_msg.control_mode)
  {
    case psyonic_hand_msgs::ControlMode::POSITION_CONTROL:
      ui.controlPositionRadioButton->setChecked(true);
      break;
    case psyonic_hand_msgs::ControlMode::VELOCITY_CONTROL:
      ui.controlVelocityRadioButton->setChecked(true);
      break;
    case psyonic_hand_msgs::ControlMode::TORQUE_CONTROL:
      ui.controlTorqueRadioButton->setChecked(true);
      break;
    case psyonic_hand_msgs::ControlMode::VOLTAGE_CONTROL:
      ui.controlVoltageRadioButton->setChecked(true);
      break;
    case psyonic_hand_msgs::ControlMode::READ_ONLY:
      ui.controlReadOnlyRadioButton->setChecked(true);
      break;
    default:
      ROS_ERROR("Unknown control mode");
      return;
  }
  updateJointCommandGUI();
}

void PsyonicHandRqtPlugin::updateReplyModeGUI()
{
  switch (reply_mode_msg.reply_mode)
  {
    case psyonic_hand_msgs::ReplyMode::REPLY_V1:
      ui.replyV1RadioButton->setChecked(true);
      break;
    case psyonic_hand_msgs::ReplyMode::REPLY_V2:
      ui.replyV2RadioButton->setChecked(true);
      break;
    case psyonic_hand_msgs::ReplyMode::REPLY_V3:
      ui.replyV3RadioButton->setChecked(true);
      break;
    default:
      ROS_ERROR("Unknown reply mode");
      return;
  }
}

void PsyonicHandRqtPlugin::updateJointCommandGUI()
{
  if (ui.controlPositionRadioButton->isChecked())
  {
    ui.velocityCommandsGroupBox->setEnabled(false);
    ui.effortCommandsGroupBox->setEnabled(false);
    ui.voltageCommandsGroupBox->setEnabled(false);
    ui.positionCommandsGroupBox->setEnabled(true);
    ui.commandStackedWidget->setCurrentIndex(0);
  }
  else if (ui.controlVelocityRadioButton->isChecked())
  {
    ui.positionCommandsGroupBox->setEnabled(false);
    ui.effortCommandsGroupBox->setEnabled(false);
    ui.voltageCommandsGroupBox->setEnabled(false);
    ui.velocityCommandsGroupBox->setEnabled(true);
    ui.commandStackedWidget->setCurrentIndex(1);
  }
  else if (ui.controlTorqueRadioButton->isChecked())
  {
    ui.positionCommandsGroupBox->setEnabled(false);
    ui.velocityCommandsGroupBox->setEnabled(false);
    ui.voltageCommandsGroupBox->setEnabled(false);
    ui.effortCommandsGroupBox->setEnabled(true);
    ui.commandStackedWidget->setCurrentIndex(2);
  }
  else if (ui.controlVoltageRadioButton->isChecked())
  {
    ui.positionCommandsGroupBox->setEnabled(false);
    ui.velocityCommandsGroupBox->setEnabled(false);
    ui.effortCommandsGroupBox->setEnabled(false);
    ui.voltageCommandsGroupBox->setEnabled(true);
    ui.commandStackedWidget->setCurrentIndex(3);
  }
  else if (ui.controlReadOnlyRadioButton->isChecked())
  {
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
}

} // namespace rqt_psyonic_hand

PLUGINLIB_EXPORT_CLASS(rqt_psyonic_hand::PsyonicHandRqtPlugin, rqt_gui_cpp::Plugin)
