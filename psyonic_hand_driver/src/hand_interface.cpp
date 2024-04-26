#include "psyonic_hand_driver/hand_interface.h"

#include <ros/ros.h>

namespace psyonic_hand_driver
{

PsyonicHand::PsyonicHand()
{
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_index("index_joint", &joint_states.index.pos, &joint_states.index.vel, &joint_states.index.eff);
  jnt_state_interface.registerHandle(state_handle_index);

  hardware_interface::JointStateHandle state_handle_middle("middle_joint", &joint_states.middle.pos, &joint_states.middle.vel, &joint_states.middle.eff);
  jnt_state_interface.registerHandle(state_handle_middle);

  hardware_interface::JointStateHandle state_handle_ring("ring_joint", &joint_states.ring.pos, &joint_states.ring.vel, &joint_states.ring.eff);
  jnt_state_interface.registerHandle(state_handle_ring);

  hardware_interface::JointStateHandle state_handle_pinky("pinky_joint", &joint_states.pinky.pos, &joint_states.pinky.vel, &joint_states.pinky.eff);
  jnt_state_interface.registerHandle(state_handle_pinky);

  hardware_interface::JointStateHandle state_handle_thumb1("thumb1_joint", &joint_states.thumb1.pos, &joint_states.thumb1.vel, &joint_states.thumb1.eff);
  jnt_state_interface.registerHandle(state_handle_thumb1);

  hardware_interface::JointStateHandle state_handle_thumb2("thumb2_joint", &joint_states.thumb2.pos, &joint_states.thumb2.vel, &joint_states.thumb2.eff);
  jnt_state_interface.registerHandle(state_handle_thumb2);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  hardware_interface::JointHandle pos_handle_index(jnt_state_interface.getHandle("index_joint"), &joint_states.index.cmd);
  jnt_pos_interface.registerHandle(pos_handle_index);

  hardware_interface::JointHandle pos_handle_middle(jnt_state_interface.getHandle("middle_joint"), &joint_states.middle.cmd);
  jnt_pos_interface.registerHandle(pos_handle_middle);

  hardware_interface::JointHandle pos_handle_ring(jnt_state_interface.getHandle("ring_joint"), &joint_states.ring.cmd);
  jnt_pos_interface.registerHandle(pos_handle_ring);

  hardware_interface::JointHandle pos_handle_pinky(jnt_state_interface.getHandle("pinky_joint"), &joint_states.pinky.cmd);
  jnt_pos_interface.registerHandle(pos_handle_pinky);

  hardware_interface::JointHandle pos_handle_thumb2(jnt_state_interface.getHandle("thumb1_joint"), &joint_states.thumb1.cmd);
  jnt_pos_interface.registerHandle(pos_handle_thumb2);

  hardware_interface::JointHandle pos_handle_thumb1(jnt_state_interface.getHandle("thumb2_joint"), &joint_states.thumb2.cmd);
  jnt_pos_interface.registerHandle(pos_handle_thumb1);

  registerInterface(&jnt_pos_interface);
}

PsyonicHand::~PsyonicHand()
{
}

bool PsyonicHand::connect(const std::string& device)
{
  return hand.connect(device);
}

bool PsyonicHand::commandReceived()
{
  JointState *joint_states_ptr = reinterpret_cast<JointState*>(&joint_states);
  bool received = false;
  for (size_t i = 0; i < NUM_HAND_JOINTS; i++)
  {
    if (std::isnan(joint_states_ptr[i].cmd)) // set to current position
    {
      joint_states_ptr[i].cmd = joint_states_ptr[i].pos;
    }
    else if (joint_states_ptr[i].cmd != joint_states_ptr[i].last_cmd)
    {
      joint_states_ptr[i].last_cmd = joint_states_ptr[i].cmd;
      received = true;
    }
  }
  return received;
}

std::unique_ptr<HandReply> PsyonicHand::sendCommand()
{
  switch (control_mode)
  {
  case ControlMode::POSITION:
    return hand.sendPositions(reply_mode_request, joint_states.index.cmd, joint_states.middle.cmd, joint_states.ring.cmd, joint_states.pinky.cmd, joint_states.thumb2.cmd, joint_states.thumb1.cmd);
  case ControlMode::READ_ONLY:
    return hand.queryStatus(reply_mode_request);
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
  std::unique_ptr<HandReply> status;
  if (!command_received)
  {
    ROS_INFO_STREAM("No command received, using read only mode");
    status = hand.queryStatus(reply_mode_request);
  }
  else
  {
    status = sendCommand();
  }

  if (!status)
  {
    ROS_ERROR("Failed to read hand status");
    return;
  }

  ROS_INFO_STREAM("Reply header: " << to_string(status->v1or2.header));

  updateJointStates(*status);

  ROS_INFO_STREAM("pos: index: " << joint_states.index.pos <<
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
                  " thumb2: " << joint_states.thumb2.eff);

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
  bool received = commandReceived();
  ROS_INFO_STREAM("Received command: " << received);
  if (received)
  {
    ROS_INFO_STREAM("Sending: " << joint_states.index.cmd << ", " << joint_states.middle.cmd << ", " << joint_states.ring.cmd << ", " << joint_states.pinky.cmd << ", " << joint_states.thumb2.cmd << ", " << joint_states.thumb1.cmd);
    std::unique_ptr<HandReply> status = sendCommand();
    if (!status)
    {
      ROS_ERROR("Failed to send hand command");
      return;
    }
    updateJointStates(*status);
    command_received = true;
  }
}

} // namespace psyonic_hand_driver