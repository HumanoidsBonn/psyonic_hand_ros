#pragma once

#include <cstdint>
#include <cmath>

namespace psyonic_hand_driver
{

static inline int16_t safeCast(double val)
{
  if (std::isnan(val))
  {
    return 0;
  }
  else if (val > 32767.0)
  {
    return 32767;
  }
  else if (val < -32768.0)
  {
    return -32768;
  }
  else
  {
    return static_cast<int16_t>(val);
  }
}

static inline double posToDegrees(int16_t pos)
{
  return static_cast<double>(pos) * 150.0 / 32767.0;
}

static inline int16_t degreesToPos(double degrees)
{
  return safeCast(degrees * 32767.0 / 150.0);
}

static inline double posToRad(int16_t pos)
{
  return static_cast<double>(pos) * M_PI / 39320.4;
}

static inline int16_t radToPos(double rad)
{
  return safeCast(rad * 39320.4 / M_PI);
}

static inline double fingerVelToDegreesPerSec(int16_t vel)
{
  return static_cast<double>(vel) * 3000.0 / 32767.0;
}

static inline int16_t degreesPerSecToFingerVel(double degrees_per_sec)
{
  return safeCast(degrees_per_sec * 32767.0 / 3000.0);
}

static inline double fingerVelToRadPerSec(int16_t vel)
{
  return static_cast<double>(vel) * M_PI / 1966.02;
}

static inline int16_t radPerSecToFingerVel(double rad_per_sec)
{
  return safeCast(rad_per_sec * 1966.02 / M_PI);
}

static inline int16_t radPerSecToRotorVel(double rad_per_sec)
{
  return safeCast(rad_per_sec * 4.0);
}

static inline double rotorVelToRadPerSec(int16_t vel)
{
  return static_cast<double>(vel) / 4.0;
}

static inline int16_t dutyCycleToVoltage(double duty_cycle)
{
  return safeCast(duty_cycle * 3546.0);
}

static inline double currentToTorque(int16_t current)
{
  constexpr double C = 620.606079; // this could be read via bluetooth
  return static_cast<double>(current) / C * 1.49 / 1000.0;
}

static inline int16_t torqueToCurrent(double torque)
{
  constexpr double C = 620.606079; // this could be read via bluetooth
  return safeCast(torque * C * 1000.0 / 1.49);
}

} // namespace psyonic_hand_driver