// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SCITOS2_MODULES__DRIVE_STATUS_BITS_HPP_
#define SCITOS2_MODULES__DRIVE_STATUS_BITS_HPP_

#include <cstdint>

namespace scitos2_modules
{

/**
 * @brief Position of every flag in the 32-bit word published by MIRA in /robot/DriveStatusPlain.
 *
 * It follows the order of the fields of \c mira::robot::DriveStatus (SCITOSDataTypes), which is
 * proprietary and is not required to build this package.
 */
enum class DriveStatusBit : uint8_t
{
  /** @brief Bit of the flag `mode_normal` of the drive status. */
  MODE_NORMAL = 0,
  /** @brief Bit of the flag `mode_forced_stopped` of the drive status. */
  MODE_FORCED_STOPPED = 1,
  /** @brief Bit of the flag `mode_freerun` of the drive status. */
  MODE_FREERUN = 2,
  /** @brief Bit of the flag `mode_safe_drive` of the drive status. */
  MODE_SAFE_DRIVE = 3,
  /** @brief Bit of the flag `mode_bumper_escape` of the drive status. */
  MODE_BUMPER_ESCAPE = 4,
  /** @brief Bit of the flag `error_sifas_communication` of the drive status. */
  ERROR_SIFAS_COMMUNICATION = 5,
  /** @brief Bit of the flag `error_sifas_internal` of the drive status. */
  ERROR_SIFAS_INTERNAL = 6,
  /** @brief Bit of the flag `emergency_stop_activated` of the drive status. */
  EMERGENCY_STOP_ACTIVATED = 7,
  /** @brief Bit of the flag `emergency_stop_status` of the drive status. */
  EMERGENCY_STOP_STATUS = 8,
  /** @brief Bit of the flag `bumper_front_activated` of the drive status. */
  BUMPER_FRONT_ACTIVATED = 9,
  /** @brief Bit of the flag `bumper_front_status` of the drive status. */
  BUMPER_FRONT_STATUS = 10,
  /** @brief Bit of the flag `bumper_rear_activated` of the drive status. */
  BUMPER_REAR_ACTIVATED = 11,
  /** @brief Bit of the flag `bumper_rear_status` of the drive status. */
  BUMPER_REAR_STATUS = 12,
  /** @brief Bit of the flag `magnetic_proximity_sensor_activated` of the drive status. */
  MAGNETIC_PROXIMITY_SENSOR_ACTIVATED = 13,
  /** @brief Bit of the flag `magnetic_proximity_sensor_status` of the drive status. */
  MAGNETIC_PROXIMITY_SENSOR_STATUS = 14,
  /** @brief Bit of the flag `external_safety_ctrl_status` of the drive status. */
  EXTERNAL_SAFETY_CTRL_STATUS = 15,
  /** @brief Bit of the flag `error_battery_low` of the drive status. */
  ERROR_BATTERY_LOW = 16,
  /** @brief Bit of the flag `error_temperature_high` of the drive status. */
  ERROR_TEMPERATURE_HIGH = 17,
  /** @brief Bit of the flag `error_stall_mode` of the drive status. */
  ERROR_STALL_MODE = 18,
  /** @brief Bit of the flag `limited_pwm_left` of the drive status. */
  LIMITED_PWM_LEFT = 19,
  /** @brief Bit of the flag `limited_pwm_right` of the drive status. */
  LIMITED_PWM_RIGHT = 20,
  /** @brief Bit of the flag `error_rotation_too_high` of the drive status. */
  ERROR_ROTATION_TOO_HIGH = 21,
  /** @brief Bit of the flag `error_motor_acceleration` of the drive status. */
  ERROR_MOTOR_ACCELERATION = 22,
  /** @brief Bit of the flag `error_motor_left` of the drive status. */
  ERROR_MOTOR_LEFT = 23,
  /** @brief Bit of the flag `error_motor_right` of the drive status. */
  ERROR_MOTOR_RIGHT = 24,
  /** @brief Bit of the flag `safety_field_rear_laser` of the drive status. */
  SAFETY_FIELD_REAR_LASER = 25,
  /** @brief Bit of the flag `safety_field_front_laser` of the drive status. */
  SAFETY_FIELD_FRONT_LASER = 26,
  /** @brief Bit of the flag `error_firmare` of the drive status. */
  ERROR_FIRMARE = 27,
  /** @brief Bit of the flag `relay_active` of the drive status. */
  RELAY_ACTIVE = 28,
  /** @brief Bit of the flag `temperature_warning` of the drive status. */
  TEMPERATURE_WARNING = 29,
  /** @brief Bit of the flag `traction_loss` of the drive status. */
  TRACTION_LOSS = 30,
  /** @brief Bit of the flag `software_safe_stop` of the drive status. */
  SOFTWARE_SAFE_STOP = 31,
};

/**
 * @brief Whether a flag of the drive status word is set.
 *
 * @param status The drive status word published by MIRA
 * @param bit The flag to check
 * @return bool True if the flag is set
 */
constexpr bool isDriveStatusBitSet(uint32_t status, DriveStatusBit bit)
{
  return (status >> static_cast<uint8_t>(bit)) & 1u;
}

}  // namespace scitos2_modules

#endif  // SCITOS2_MODULES__DRIVE_STATUS_BITS_HPP_
