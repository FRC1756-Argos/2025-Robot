/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <argos_lib/general/generic_debouncer.h>
#include <argos_lib/subsystems/swappable_controllers_subsystem.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "commands/autonomous/autonomous_nothing.h"
#include "subsystems/simple_led_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"
#include "subsystems/vision_subsystem.h"
#include "utils/auto_selector.h"

/**
 * @brief  Command-based is a "declarative" paradigm, very little robot logic should
 *         actually be handled in the {@link Robot} periodic methods (other than the
 *         scheduler calls).  Instead, the structure of the robot (including subsystems,
 *         commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  /// @brief Called once when robot is disabled
  void Disable();

  /// @brief Called once when robot is enabled
  void Enable();

  /// @brief Called when the alliance is changed
  void AllianceChanged();

  void SetLedsConnectedBrightness(bool connected);

 private:
  // Interpolation of controller inputs. Used for making the inputs non-linear, allowing finer control of how the robot responds to the joystick.
  argos_lib::InterpolationMap<decltype(controllerMap::driveSpeed.front().inVal), controllerMap::driveSpeed.size()>
      m_driveSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::driveRotSpeed.front().inVal), controllerMap::driveRotSpeed.size()>
      m_driveRotSpeed;

  const argos_lib::RobotInstance m_instance;

  // The robot's subsystems are defined here...
  argos_lib::SwappableControllersSubsystem m_controllers;
  SwerveDriveSubsystem m_swerveDrive;
  SimpleLedSubsystem m_ledSubSystem;
  VisionSubsystem m_visionSubSystem;

  // Autonomous
  AutonomousNothing m_autoNothing;

  AutoSelector m_autoSelector;

  bool m_transitionedFromAuto;  ///< True indicates latest enable was during autonomous

  void ConfigureBindings();
};
