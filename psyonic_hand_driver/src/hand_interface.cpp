#include "psyonic_hand_driver/hand_interface.h"

#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>

namespace psyonic_hand_driver
{

PsyonicHand::PsyonicHand()
{
  // connect and register the joint state interface
  for (size_t i = 0; i < NUM_HAND_JOINTS; i++)
  {
    hardware_interface::JointStateHandle state_handle(JOINT_NAMES[i], &joint_states[i].pos, &joint_states[i].vel, &joint_states[i].eff);
    jnt_state_interface.registerHandle(state_handle);
  }

  registerInterface(&jnt_state_interface);

  for (size_t i = 0; i < NUM_HAND_JOINTS; i++)
  {
    hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(JOINT_NAMES[i]), &joint_states[i].cmd_pos);
    jnt_pos_interface.registerHandle(pos_handle);
    hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(JOINT_NAMES[i]), &joint_states[i].cmd_vel);
    jnt_vel_interface.registerHandle(vel_handle);
    hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(JOINT_NAMES[i]), &joint_states[i].cmd_eff);
    jnt_eff_interface.registerHandle(eff_handle);
  }

  registerInterface(&jnt_pos_interface);
  registerInterface(&jnt_vel_interface);
  registerInterface(&jnt_eff_interface);
}

PsyonicHand::~PsyonicHand()
{
}

bool PsyonicHand::setControlInterface(ControlInterface interface)
{
  if (interface == ControlInterface::SERIAL)
  {
    control_interface = ControlInterface::SERIAL;
    if (!hand_ble.sendCommand(to_command(PlotModeBLE::DISABLE)))
    {
      ROS_ERROR("Failed to disable plot mode");
      return false;
    }
    ble_plot_mode = PlotModeBLE::DISABLE;
    return true;
  }
  else if (interface == ControlInterface::BLE)
  {
    if (!hand_ble.isConnected())
    {
      ROS_ERROR("Cannot set control interface to BLE; hand is not connected");
      return false;
    }
    control_interface = ControlInterface::BLE;
    if (!hand_ble.sendCommand(to_command(PlotModeBLE::FINGER_POSITION)))
    {
      ROS_ERROR("Failed to set plot mode to finger position");
      return false;
    }
    ble_plot_mode = PlotModeBLE::FINGER_POSITION;
    return true;
  }
  return false;
}

bool PsyonicHand::setControlMode(ControlMode mode)
{
  if (mode == ControlMode::POSITION || mode == ControlMode::VELOCITY || mode == ControlMode::TORQUE || mode == ControlMode::VOLTAGE || mode == ControlMode::READ_ONLY)
  {
    control_mode = mode;
    return true;
  }
  return false;
}

bool PsyonicHand::setReplyMode(ReplyMode mode)
{
  if (mode == ReplyMode::V1 || mode == ReplyMode::V2 || mode == ReplyMode::V3)
  {
    reply_mode_request = mode;
    return true;
  }
  return false;
}

void PsyonicHand::setVoltageCommand(double index, double middle, double ring, double pinky, double thumb1, double thumb2)
{
  joint_states.index.cmd_vol = index;
  joint_states.middle.cmd_vol = middle;
  joint_states.ring.cmd_vol = ring;
  joint_states.pinky.cmd_vol = pinky;
  joint_states.thumb1.cmd_vol = thumb1;
  joint_states.thumb2.cmd_vol = thumb2;
}

void PsyonicHand::handMessageReceivedBLE(SimpleBLE::ByteArray payload)
{
  if (ble_plot_mode == PlotModeBLE::FINGER_POSITION)
  {
    if (payload.size() != 6 * sizeof(float))
    {
      ROS_ERROR_STREAM("Received invalid message: " << payload.size() << " bytes");
      return;
    }
    float* data = reinterpret_cast<float*>(payload.data());
    joint_states.index.pos = data[0] * M_PI / 180.0;
    joint_states.middle.pos = data[1] * M_PI / 180.0;
    joint_states.ring.pos = data[2] * M_PI / 180.0;
    joint_states.pinky.pos = data[3] * M_PI / 180.0;
    joint_states.thumb2.pos = data[4] * M_PI / 180.0;
    joint_states.thumb1.pos = data[5] * M_PI / 180.0;
    ble_new_data_received = true;
  }
  else if (ble_plot_mode == PlotModeBLE::DISABLE)
  {
    ROS_ERROR("Received message in disabled plot mode");
  }
  else
  {
    ROS_ERROR("Unknown plot mode");
  }
}

void PsyonicHand::writePositionBLE(double index, double middle, double ring, double pinky, double thumb_flexor, double thumb_rotator)
{
  hand_ble.sendPositionCommand(index, middle, ring, pinky, thumb_flexor, thumb_rotator);
}

bool PsyonicHand::connectSerial(const std::string &port)
{
  return hand_serial.connect(port);
}

bool PsyonicHand::connectSerialById(const std::string &id)
{
  return hand_serial.connectById(id);
}

bool PsyonicHand::connectBLE(const ros::Duration &timeout)
{
  hand_ble.startScanForHand();
  bool hand_found = false;
  ros::Time start = ros::Time::now();
  while (ros::ok() && ros::Time::now() - start < timeout)
  {
    ROS_INFO_THROTTLE(1, "Scanning for hand...");
    hand_found = hand_ble.deviceFound();
    if (hand_found)
    {
      ROS_INFO("Found hand");
      break;
    }
  }
  hand_ble.stopScanForHand();
  if (!hand_found)
  {
    ROS_ERROR_STREAM("Failed to find hand");
    return false;
  }
  bool connected = hand_ble.connect();
  if (!connected)
  {
    ROS_ERROR("Failed to connect to hand");
    return false;
  }
  bool receive = hand_ble.setReceiverCallback(std::bind(&PsyonicHand::handMessageReceivedBLE, this, std::placeholders::_1));
  if (!receive)
  {
    ROS_ERROR("Failed to set receiver callback");
    return false;
  }
  return true;
}

bool PsyonicHand::disconnectSerial()
{
  return hand_serial.disconnect();

}

bool PsyonicHand::disconnectBLE()
{
  return hand_ble.disconnect();
}

std::unique_ptr<HandReply> PsyonicHand::sendCommand()
{
  switch (control_mode)
  {
  case ControlMode::POSITION:
    return hand_serial.sendPositions(reply_mode_request, joint_states.index.cmd_pos, joint_states.middle.cmd_pos, joint_states.ring.cmd_pos, joint_states.pinky.cmd_pos, joint_states.thumb2.cmd_pos, joint_states.thumb1.cmd_pos);
  case ControlMode::VELOCITY:
    return hand_serial.sendVelocities(reply_mode_request, joint_states.index.cmd_vel, joint_states.middle.cmd_vel, joint_states.ring.cmd_vel, joint_states.pinky.cmd_vel, joint_states.thumb2.cmd_vel, joint_states.thumb1.cmd_vel);
  case ControlMode::TORQUE:
    return hand_serial.sendTorque(reply_mode_request, joint_states.index.cmd_eff, joint_states.middle.cmd_eff, joint_states.ring.cmd_eff, joint_states.pinky.cmd_eff, joint_states.thumb2.cmd_eff, joint_states.thumb1.cmd_eff);
  case ControlMode::VOLTAGE:
    return hand_serial.sendVoltage(reply_mode_request, joint_states.index.cmd_vol, joint_states.middle.cmd_vol, joint_states.ring.cmd_vol, joint_states.pinky.cmd_vol, joint_states.thumb2.cmd_vol, joint_states.thumb1.cmd_vol);
  case ControlMode::READ_ONLY:
    return hand_serial.queryStatus(reply_mode_request);
  default:
    ROS_ERROR("Unknown control mode");
    return nullptr;
  }
}

void PsyonicHand::updateJointStates(const HandReply& reply)
{
  ReplyMode reply_mode = reply.replyMode();
  if (reply_mode == ReplyMode::V3)
  {
    auto decoded = reply.v3.decode();
    joint_states.index.pos = decoded->index_position;
    joint_states.middle.pos = decoded->middle_position;
    joint_states.ring.pos = decoded->ring_position;
    joint_states.pinky.pos = decoded->pinky_position;
    joint_states.thumb2.pos = decoded->thumb_flexor_position;
    joint_states.thumb1.pos = decoded->thumb_rotator_position;

    joint_states.index.eff = decoded->index_torque;
    joint_states.middle.eff = decoded->middle_torque;
    joint_states.ring.eff = decoded->ring_torque;
    joint_states.pinky.eff = decoded->pinky_torque;
    joint_states.thumb2.eff = decoded->thumb_flexor_torque;
    joint_states.thumb1.eff = decoded->thumb_rotator_torque;

    joint_states.index.vel = decoded->index_velocity;
    joint_states.middle.vel = decoded->middle_velocity;
    joint_states.ring.vel = decoded->ring_velocity;
    joint_states.pinky.vel = decoded->pinky_velocity;
    joint_states.thumb2.vel = decoded->thumb_flexor_velocity;
    joint_states.thumb1.vel = decoded->thumb_rotator_velocity;
  }
  else
  {
    auto decoded = reply.v1or2.decode();
    joint_states.index.pos = decoded->index_position;
    joint_states.middle.pos = decoded->middle_position;
    joint_states.ring.pos = decoded->ring_position;
    joint_states.pinky.pos = decoded->pinky_position;
    joint_states.thumb2.pos = decoded->thumb_flexor_position;
    joint_states.thumb1.pos = decoded->thumb_rotator_position;

    if (reply_mode == ReplyMode::V1) // torque -> effort
    {
      joint_states.index.eff = decoded->index_torque_or_velocity;
      joint_states.middle.eff = decoded->middle_torque_or_velocity;
      joint_states.ring.eff = decoded->ring_torque_or_velocity;
      joint_states.pinky.eff = decoded->pinky_torque_or_velocity;
      joint_states.thumb2.eff = decoded->thumb_flexor_torque_or_velocity;
      joint_states.thumb1.eff = decoded->thumb_rotator_torque_or_velocity;
    }
    else // velocity -> velocity
    {
      joint_states.index.vel = decoded->index_torque_or_velocity;
      joint_states.middle.vel = decoded->middle_torque_or_velocity;
      joint_states.ring.vel = decoded->ring_torque_or_velocity;
      joint_states.pinky.vel = decoded->pinky_torque_or_velocity;
      joint_states.thumb2.vel = decoded->thumb_flexor_torque_or_velocity;
      joint_states.thumb1.vel = decoded->thumb_rotator_torque_or_velocity;
    }
  }
}

void PsyonicHand::read(const ros::Time& time, const ros::Duration& period)
{
  if (control_interface == ControlInterface::SERIAL)
  {
    if (!status) // first read / last command failed
    {
      status = hand_serial.queryStatus(reply_mode_request);
    }

    if (!status)
    {
      ROS_ERROR("Failed to read hand status");
      return;
    }

    updateJointStates(*status); // use status obtained from last command sent
  }
  else
  {
    ros::Time start = ros::Time::now();
    while (ros::Time::now() - start < ros::Duration(0.25) && !ble_new_data_received);
    if (!ble_new_data_received)
    {
      ROS_ERROR("Failed to receive new data from BLE");
      return;
    }
    ble_new_data_received = false;
  }
  
  // in BLE mode, status is updated in handMessageReceivedBLE

  if (control_mode == ControlMode::READ_ONLY)
  {
    for (size_t i = 0; i < NUM_HAND_JOINTS; i++)
    {
      joint_states[i].cmd_pos = joint_states[i].pos;
    }
  }

  /*ROS_INFO_STREAM("pos: index: " << joint_states.index.pos <<
                  " middle: " << joint_states.middle.pos <<
                  " ring: " << joint_states.ring.pos <<
                  " pinky: " << joint_states.pinky.pos <<
                  " thumb1: " << joint_states.thumb1.pos <<
                  " thumb2: " << joint_states.thumb2.pos);

  ROS_INFO_STREAM("vel: index: " << joint_states.index.vel <<
                  " middle: " << joint_states.middle.vel <<
                  " ring: " << joint_states.ring.vel <<
                  " pinky: " << joint_states.pinky.vel <<
                  " thumb1: " << joint_states.thumb1.vel <<
                  " thumb2: " << joint_states.thumb2.vel);

  ROS_INFO_STREAM("eff: index: " << joint_states.index.eff <<
                  " middle: " << joint_states.middle.eff <<
                  " ring: " << joint_states.ring.eff <<
                  " pinky: " << joint_states.pinky.eff <<
                  " thumb1: " << joint_states.thumb1.eff <<
                  " thumb2: " << joint_states.thumb2.eff);*/

  /*auto touch = status->v1or2.unpackTouchSensorData()->decode();
  if (!touch)
  {
    ROS_ERROR("Failed to unpack touch data");
  }
  else
  {
    ROS_INFO_STREAM("index: " << touch->index_site0 << ", " << touch->index_site1 << ", " << touch->index_site2 << ", " << touch->index_site3 << ", " << touch->index_site4 << ", " << touch->index_site5);
    ROS_INFO_STREAM("middle: " << touch->middle_site0 << ", " << touch->middle_site1 << ", " << touch->middle_site2 << ", " << touch->middle_site3 << ", " << touch->middle_site4 << ", " << touch->middle_site5);
    ROS_INFO_STREAM("ring: " << touch->ring_site0 << ", " << touch->ring_site1 << ", " << touch->ring_site2 << ", " << touch->ring_site3 << ", " << touch->ring_site4 << ", " << touch->ring_site5);
    ROS_INFO_STREAM("pinky: " << touch->pinky_site0 << ", " << touch->pinky_site1 << ", " << touch->pinky_site2 << ", " << touch->pinky_site3 << ", " << touch->pinky_site4 << ", " << touch->pinky_site5);
    ROS_INFO_STREAM("thumb: " << touch->thumb_site0 << ", " << touch->thumb_site1 << ", " << touch->thumb_site2 << ", " << touch->thumb_site3 << ", " << touch->thumb_site4 << ", " << touch->thumb_site5 << "\n");
  }*/
}

void PsyonicHand::write(const ros::Time& time, const ros::Duration& period)
{
  if (control_interface == ControlInterface::BLE)
  {
    if (control_mode == ControlMode::POSITION)
    {
      hand_ble.sendPositionCommand(joint_states.index.cmd_pos, joint_states.middle.cmd_pos, joint_states.ring.cmd_pos, joint_states.pinky.cmd_pos, joint_states.thumb2.cmd_pos, joint_states.thumb1.cmd_pos);
    }
    else
    {
      ROS_ERROR_THROTTLE(1, "Only position control is supported over BLE; using serial");
    }
  }
  else
  {
    status = sendCommand();
    if (!status)
    {
      ROS_ERROR("Failed to send hand command");
      return;
    }
  }
}

} // namespace psyonic_hand_driver

PLUGINLIB_EXPORT_CLASS(psyonic_hand_driver::PsyonicHand, hardware_interface::RobotHW)