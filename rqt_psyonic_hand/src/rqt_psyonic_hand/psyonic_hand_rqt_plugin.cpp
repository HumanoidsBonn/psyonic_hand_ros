#include "rqt_psyonic_hand/psyonic_hand_rqt_plugin.h"

#include <pluginlib/class_list_macros.h>

#include <std_msgs/Float64.h>

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

  // Match sliders and spinboxes
  joint_slider_spinbox_map[ui.indexJointPositionSlider] = ui.indexJointPostionSpinBox;
  joint_spinbox_slider_map[ui.indexJointPostionSpinBox] = ui.indexJointPositionSlider;
  joint_slider_spinbox_map[ui.middleJointPositionSlider] = ui.middleJointPostionSpinBox;
  joint_spinbox_slider_map[ui.middleJointPostionSpinBox] = ui.middleJointPositionSlider;
  joint_slider_spinbox_map[ui.ringJointPositionSlider] = ui.ringJointPostionSpinBox;
  joint_spinbox_slider_map[ui.ringJointPostionSpinBox] = ui.ringJointPositionSlider;
  joint_slider_spinbox_map[ui.pinkyJointPositionSlider] = ui.pinkyJointPostionSpinBox;
  joint_spinbox_slider_map[ui.pinkyJointPostionSpinBox] = ui.pinkyJointPositionSlider;
  joint_slider_spinbox_map[ui.thumb1JointPositionSlider] = ui.thumb1JointPostionSpinBox;
  joint_spinbox_slider_map[ui.thumb1JointPostionSpinBox] = ui.thumb1JointPositionSlider;
  joint_slider_spinbox_map[ui.thumb2JointPositionSlider] = ui.thumb2JointPostionSpinBox;
  joint_spinbox_slider_map[ui.thumb2JointPostionSpinBox] = ui.thumb2JointPositionSlider;

  // Match sliders and publishers
  joint_pub_map[ui.indexJointPositionSlider] = &index_joint_pub;
  joint_pub_map[ui.middleJointPositionSlider] = &middle_joint_pub;
  joint_pub_map[ui.ringJointPositionSlider] = &ring_joint_pub;
  joint_pub_map[ui.pinkyJointPositionSlider] = &pinky_joint_pub;
  joint_pub_map[ui.thumb1JointPositionSlider] = &thumb1_joint_pub;
  joint_pub_map[ui.thumb2JointPositionSlider] = &thumb2_joint_pub;

  index_joint_pub = getNodeHandle().advertise<std_msgs::Float64>("/hand_interface_node/index_joint_controller/command", 1);
  middle_joint_pub = getNodeHandle().advertise<std_msgs::Float64>("/hand_interface_node/middle_joint_controller/command", 1);
  ring_joint_pub = getNodeHandle().advertise<std_msgs::Float64>("/hand_interface_node/ring_joint_controller/command", 1);
  pinky_joint_pub = getNodeHandle().advertise<std_msgs::Float64>("/hand_interface_node/pinky_joint_controller/command", 1);
  thumb1_joint_pub = getNodeHandle().advertise<std_msgs::Float64>("/hand_interface_node/thumb1_joint_controller/command", 1);
  thumb2_joint_pub = getNodeHandle().advertise<std_msgs::Float64>("/hand_interface_node/thumb2_joint_controller/command", 1);

  connect(ui.indexJointPositionSlider, &QSlider::sliderMoved, this, &PsyonicHandRqtPlugin::jointPositionCommandSliderMoved);
  connect(ui.indexJointPostionSpinBox, &QDoubleSpinBox::editingFinished, this, &PsyonicHandRqtPlugin::jointPositionCommandSpinBoxEdited);
  connect(ui.middleJointPositionSlider, &QSlider::sliderMoved, this, &PsyonicHandRqtPlugin::jointPositionCommandSliderMoved);
  connect(ui.middleJointPostionSpinBox, &QDoubleSpinBox::editingFinished, this, &PsyonicHandRqtPlugin::jointPositionCommandSpinBoxEdited);
  connect(ui.ringJointPositionSlider, &QSlider::sliderMoved, this, &PsyonicHandRqtPlugin::jointPositionCommandSliderMoved);
  connect(ui.ringJointPostionSpinBox, &QDoubleSpinBox::editingFinished, this, &PsyonicHandRqtPlugin::jointPositionCommandSpinBoxEdited);
  connect(ui.pinkyJointPositionSlider, &QSlider::sliderMoved, this, &PsyonicHandRqtPlugin::jointPositionCommandSliderMoved);
  connect(ui.pinkyJointPostionSpinBox, &QDoubleSpinBox::editingFinished, this, &PsyonicHandRqtPlugin::jointPositionCommandSpinBoxEdited);
  connect(ui.thumb1JointPositionSlider, &QSlider::sliderMoved, this, &PsyonicHandRqtPlugin::jointPositionCommandSliderMoved);
  connect(ui.thumb1JointPostionSpinBox, &QDoubleSpinBox::editingFinished, this, &PsyonicHandRqtPlugin::jointPositionCommandSpinBoxEdited);
  connect(ui.thumb2JointPositionSlider, &QSlider::sliderMoved, this, &PsyonicHandRqtPlugin::jointPositionCommandSliderMoved);
  connect(ui.thumb2JointPostionSpinBox, &QDoubleSpinBox::editingFinished, this, &PsyonicHandRqtPlugin::jointPositionCommandSpinBoxEdited);
}

void PsyonicHandRqtPlugin::shutdownPlugin()
{
  // unregister all publishers here
  index_joint_pub.shutdown();
  middle_joint_pub.shutdown();
  ring_joint_pub.shutdown();
  pinky_joint_pub.shutdown();
  thumb1_joint_pub.shutdown();
  thumb2_joint_pub.shutdown();
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

void PsyonicHandRqtPlugin::publishJointPositionCommand(ros::Publisher* pub, double value)
{
  std_msgs::Float64 msg;
  msg.data = value * M_PI / 180.0;
  pub->publish(msg);
}

void PsyonicHandRqtPlugin::jointPositionCommandSliderMoved(int value)
{
  QSlider *slider = qobject_cast<QSlider*>(sender());
  joint_slider_spinbox_map[slider]->setValue(static_cast<double>(value));
  publishJointPositionCommand(joint_pub_map[slider], static_cast<double>(value));
}

void PsyonicHandRqtPlugin::jointPositionCommandSpinBoxEdited()
{
  QDoubleSpinBox *spinbox = qobject_cast<QDoubleSpinBox*>(sender());
  double value = spinbox->value();
  QSlider *slider = joint_spinbox_slider_map[spinbox];
  slider->setValue(static_cast<int>(value));
  publishJointPositionCommand(joint_pub_map[slider], value);
}

} // namespace rqt_psyonic_hand

PLUGINLIB_EXPORT_CLASS(rqt_psyonic_hand::PsyonicHandRqtPlugin, rqt_gui_cpp::Plugin)
