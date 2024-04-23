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

  hardware_interface::JointHandle pos_handle_thumb1(jnt_state_interface.getHandle("thumb1_joint"), &joint_states.thumb1.cmd);
  jnt_pos_interface.registerHandle(pos_handle_thumb1);

  hardware_interface::JointHandle pos_handle_thumb2(jnt_state_interface.getHandle("thumb2_joint"), &joint_states.thumb2.cmd);
  jnt_pos_interface.registerHandle(pos_handle_thumb2);

  registerInterface(&jnt_pos_interface);
}

PsyonicHand::~PsyonicHand()
{
}

bool PsyonicHand::connect(const std::string& device)
{
  return hand.connect(device);
}

void PsyonicHand::read(const ros::Time& time, const ros::Duration& period)
{
  std::unique_ptr<HandReplyV1or2> status = hand.queryStatusV1();

  if (!status)
  {
    ROS_ERROR("Failed to read hand status");
    return;
  }

  joint_states.index.pos = hand.posToRad(status->index_position);
  joint_states.middle.pos = hand.posToRad(status->middle_position);
  joint_states.ring.pos = hand.posToRad(status->ring_position);
  joint_states.pinky.pos = hand.posToRad(status->pinky_position);
  joint_states.thumb1.pos = hand.posToRad(status->thumb_flexor_position);
  joint_states.thumb2.pos = hand.posToRad(status->thumb_rotator_position);

  ROS_INFO_STREAM("index: " << joint_states.index.pos <<
                  " middle: " << joint_states.middle.pos <<
                  " ring: " << joint_states.ring.pos <<
                  " pinky: " << joint_states.pinky.pos <<
                  " thumb1: " << joint_states.thumb1.pos <<
                  " thumb2: " << joint_states.thumb2.pos);
}

void PsyonicHand::write(const ros::Time& time, const ros::Duration& period)
{
}

} // namespace psyonic_hand_driver