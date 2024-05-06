#pragma once

#include "hand_serial.h"
#include "hand_ble.h"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace psyonic_hand_driver
{

enum class ControlInterface
{
  SERIAL = 0,
  BLE = 1
};

struct JointState
{
  double pos = 0;
  double vel = 0;
  double eff = 0;
  double cmd_pos = 0;
  double cmd_vel = 0;
  double cmd_eff = 0;
  double cmd_vol = 0;
};

struct HandJointStates
{
  JointState index;
  JointState middle;
  JointState ring;
  JointState pinky;
  JointState thumb1;
  JointState thumb2;

  JointState& operator[](size_t i)
  {
    return *(&index + i);
  }

  const JointState& operator[](size_t i) const
  {
    return *(&index + i);
  }
};

static constexpr size_t NUM_HAND_JOINTS = 6;

static constexpr std::array<const char*, NUM_HAND_JOINTS> JOINT_NAMES = {
  "index_joint",
  "middle_joint",
  "ring_joint",
  "pinky_joint",
  "thumb1_joint",
  "thumb2_joint"
};

static_assert(sizeof(HandJointStates) == NUM_HAND_JOINTS * sizeof(JointState), "HandJointStates size is not correct");

class PsyonicHand : public hardware_interface::RobotHW
{
private:
  HandJointStates joint_states;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;

  ControlInterface control_interface = ControlInterface::SERIAL;
  ControlMode control_mode = ControlMode::READ_ONLY;
  ReplyMode reply_mode_request = ReplyMode::V3;
  HandSerial hand_serial;
  HandBLE hand_ble;
  PlotModeBLE ble_plot_mode = PlotModeBLE::DISABLE;
  bool ble_new_data_received = false;
  bool command_received = false;

  std::unique_ptr<HandReply> status = nullptr;

public:
  PsyonicHand();
  virtual ~PsyonicHand();

  ControlInterface getControlInterface() const { return control_interface; }
  ControlMode getControlMode() const { return control_mode; }
  ReplyMode getReplyMode() const { return reply_mode_request; }

  bool setControlInterface(ControlInterface interface);
  bool setControlMode(ControlMode mode);
  bool setReplyMode(ReplyMode mode);

  void setVoltageCommand(double index, double middle, double ring, double pinky, double thumb_flexor, double thumb_rotator);

  void handMessageReceivedBLE(SimpleBLE::ByteArray payload);
  void writePositionBLE(double index, double middle, double ring, double pinky, double thumb_flexor, double thumb_rotator);

  bool connectSerial(const std::string& device);
  bool connectBLE(const ros::Duration &timeout);

  bool disconnectSerial();
  bool disconnectBLE();

  /**
   * \brief Send command, depending on control mode
   *
   * \return Reply from the hand, or nullptr if an error occurred
   */
  std::unique_ptr<HandReply> sendCommand();

  /** \brief Update joint states from a hand reply. Automatically detects the reply mode.
   *
   * \param reply The hand reply
   */
  void updateJointStates(const HandReply& reply);

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