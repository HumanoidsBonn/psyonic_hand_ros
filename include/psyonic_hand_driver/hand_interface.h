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
  double cmd = std::numeric_limits<double>::quiet_NaN();
  double last_cmd = std::numeric_limits<double>::quiet_NaN();
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

static constexpr size_t NUM_HAND_JOINTS = 6;

static_assert(sizeof(HandJointStates) == NUM_HAND_JOINTS * sizeof(JointState), "HandJointStates size is not correct");

class PsyonicHand : public hardware_interface::RobotHW
{
private:
  HandJointStates joint_states;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;

  HandSerial hand;

  bool position_control_mode = false;

public:
  PsyonicHand();
  virtual ~PsyonicHand();

  bool connect(const std::string& device);

  /** \brief Check if a command has been received, adjust if necessary
   *
   * \return true if a command has been received
   */
  bool commandReceived();

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