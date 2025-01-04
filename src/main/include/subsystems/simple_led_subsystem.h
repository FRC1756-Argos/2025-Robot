/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/general/log.h>
#include <constants/field_points.h>
#include <ctre/phoenix/led/CANdle.h>
#include <frc/DriverStation.h>
#include <frc/util/Color.h>
#include <frc2/command/SubsystemBase.h>

#include <chrono>
#include <functional>

#include "argos_lib/config/config_types.h"
#include "argos_lib/general/color.h"

enum class LedGroup { BACK, FRONT };
enum class LedStrip { BackLeft, BackRight, Front };
enum class AlignLedStatus { NoTarget, FlashLeft, FlashRight, Aligned };

class SimpleLedSubsystem : public frc2::SubsystemBase {
 public:
  explicit SimpleLedSubsystem(argos_lib::RobotInstance instance);

  void Enable();
  void Disable();

  void SetLedsConnectedBrightness(bool connected);

  void SetDisableAnimation(std::function<void()> animationFunction);

  /// @brief Sets group of leds to given color
  /// @param group The group of leds to set
  /// @param color an ArgosColor to set the LEDs too
  void SetLedGroupColor(LedGroup group, argos_lib::ArgosColor color, bool restorable = true);
  void SetLedStripColor(LedStrip strip, argos_lib::ArgosColor color, bool restorable = true);

  /// @brief Sets all led groups to a given color
  /// @param color an ArgosColor to set the LEDs too
  void SetAllGroupsColor(argos_lib::ArgosColor color, bool restorable = true);

  void SetAllGroupsFade(argos_lib::ArgosColor color, bool restorable = true);

  void SetAllGroupsFlash(argos_lib::ArgosColor color, bool restorable = true);

  void FlashStrip(LedStrip strip, argos_lib::ArgosColor color, bool restorable = true);

  void SetAllGroupsLarson(argos_lib::ArgosColor color, bool restorable = true);

  argos_lib::ArgosColor GetAllianceColor();

  /// @brief Set all groups of LEDs to the alliance color
  void SetAllGroupsAllianceColor(bool fade, bool restorable = true);

  void StopAllAnimations(bool restorable = true);

  /// @brief Turn off all LEDs
  void SetAllGroupsOff(bool restorable = true);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void FireEverywhere(bool restorable = true);

  void ColorSweep(argos_lib::ArgosColor color, bool correctGamma = true, bool restorable = true);

  void TemporaryAnimate(std::function<void()> animationFunction, units::millisecond_t duration);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  std::optional<ctre::phoenix::led::CANdle> m_CANdle;
  argos_lib::ArgosLogger m_log;
  bool m_enabled;
  bool m_hasBeenConnected;

  std::function<void()> m_disableUpdateFunction;
  std::function<void()> m_ledUpdateFunction;
  std::optional<std::function<void(void)>> m_restoreAnimationFunction;
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;
  units::millisecond_t m_temporaryDuration;

  frc::DriverStation::Alliance m_latestAlliance;

  constexpr static int startIndex_backLeft = 8;    ///< Address of first LED in strip
  constexpr static int length_backLeft = 23;       ///< Number of LEDs in strip
  constexpr static bool inverted_backLeft = true;  ///< true indicates first index is at top of tower

  constexpr static int startIndex_backRight = 31;
  constexpr static int length_backRight = 23;
  constexpr static bool inverted_backRight = true;

  constexpr static int startIndex_front = 54;
  constexpr static int length_front = 9;
  constexpr static bool inverted_front = true;
};
