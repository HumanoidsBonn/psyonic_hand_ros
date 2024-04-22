#pragma once

#include "hand_serial.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace psyonic_hand_driver
{

struct JointState
{
  double pos;
  double vel;
  double eff;
  double cmd;
};

struct HandJointStates
{
  JointState index;
  JointState middle;
  JointState ring;
  JointState pinky;
  JointState thumb1;
  JointState thumb2;
};

class PsyonicHand : public hardware_interface::RobotHW
{
private:
  HandJointStates joint_states;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;

  HandSerial serial;

public:
  PsyonicHand();
  virtual ~PsyonicHand();

  /** \brief Read data from the robot hardware.
   *
   * The read method is part of the control loop cycle (\ref read, update, \ref write) 
   * and is used to populate the robot state from the robot's hardware resources
   * (joints, sensors, actuators). This method should be called before 
   * controller_manager::ControllerManager::update() and \ref write.
   * 
   * \note The name \ref read refers to reading state from the hardware.
   * This complements \ref write, which refers to writing commands to the hardware.
   *
   * Querying WallTime inside \ref read is not realtime safe. The parameters
   * \ref time and \ref period make it possible to inject time from a realtime source.
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref read
   */
  virtual void read(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

  /** \brief Write commands to the robot hardware.
   * 
   * The write method is part of the control loop cycle (\ref read, update, \ref write) 
   * and is used to send out commands to the robot's hardware 
   * resources (joints, actuators). This method should be called after 
   * \ref read and controller_manager::ControllerManager::update.
   * 
   * \note The name \ref write refers to writing commands to the hardware.
   * This complements \ref read, which refers to reading state from the hardware.
   *
   * Querying WallTime inside \ref write is not realtime safe. The parameters
   * \ref time and \ref period make it possible to inject time from a realtime source.
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref write
   */
  virtual void write(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;
};

} // namespace psyonic_hand_driver