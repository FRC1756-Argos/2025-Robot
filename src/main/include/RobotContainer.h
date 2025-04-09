/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <argos_lib/general/generic_debouncer.h>
#include <argos_lib/subsystems/swappable_controllers_subsystem.h>
#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "commands/autonomous/autonomous_L1L4_EDC.h"
#include "commands/autonomous/autonomous_L1L4_JKL.h"
#include "commands/autonomous/autonomous_L1_FE.h"
#include "commands/autonomous/autonomous_L1_GH.h"
#include "commands/autonomous/autonomous_L1_IJ.h"
#include "commands/autonomous/autonomous_L1_J_L4_L.h"
#include "commands/autonomous/autonomous_L4_EDC.h"
#include "commands/autonomous/autonomous_L4_G.h"
#include "commands/autonomous/autonomous_L4_JKL.h"
#include "commands/autonomous/autonomous_choreo_test.h"
#include "commands/autonomous/autonomous_forward.h"
#include "commands/autonomous/autonomous_nothing.h"
#include "controls/operator_controller.h"
#include "subsystems/climber_subsystem.h"
#include "subsystems/elevator_subsystem.h"
#include "subsystems/intake_subsystem.h"
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

  auto& GetSwerveDrive() { return m_swerveDrive; }

 private:
  // Interpolation of controller inputs. Used for making the inputs non-linear, allowing finer control of how the robot responds to the joystick.
  argos_lib::InterpolationMap<decltype(controllerMap::driveSpeed.front().inVal), controllerMap::driveSpeed.size()>
      m_driveSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::driveSpeed_placing.front().inVal),
                              controllerMap::driveSpeed_placing.size()>
      m_driveSpeedMap_placing;
  argos_lib::InterpolationMap<decltype(controllerMap::driveSpeed_intake.front().inVal),
                              controllerMap::driveSpeed_intake.size()>
      m_driveSpeedMap_intake;
  argos_lib::InterpolationMap<decltype(controllerMap::driveSpeed_intakeAlgae.front().inVal),
                              controllerMap::driveSpeed_intakeAlgae.size()>
      m_driveSpeedMap_intakeAlgae;
  argos_lib::InterpolationMap<decltype(controllerMap::driveRotSpeed.front().inVal), controllerMap::driveRotSpeed.size()>
      m_driveRotSpeed;
  argos_lib::InterpolationMap<decltype(controllerMap::driveRotSpeed_placing.front().inVal),
                              controllerMap::driveRotSpeed_placing.size()>
      m_driveRotSpeed_placing;

  const argos_lib::RobotInstance m_instance;

  // The robot's subsystems are defined here...
  argos_lib::SwappableControllersSubsystem m_controllers;
  OperatorController m_macropadController;
  SwerveDriveSubsystem m_swerveDrive;
  SimpleLedSubsystem m_ledSubSystem;
  VisionSubsystem m_visionSubSystem;
  ElevatorSubsystem m_elevatorSubSystem;
  ClimberSubsystem m_climberSubSystem;
  IntakeSubsystem m_intakeSubSystem;

  // Autonomous
  AutonomousNothing m_autoNothing;
  AutonomousL1FE m_autoL1FE;
  AutonomousChoreoTest m_autoChoreoTest;
  AutonomousForward m_autoForward;
  AutonomousL1GH m_autoL1GH;
  AutonomousL1IJ m_autoL1IJ;
  AutonomousL4G m_autoL4G;
  AutonomousL4EDC m_autoL4EDC;
  AutonomousL4JKL m_autoL4JKL;
  AutonomousL1JL4L m_autoL1JL4L;
  AutonomousL1L4EDC m_autoL1L4EDC;
  AutonomousL1L4JKL m_autoL1L4JKL;

  AutoSelector m_autoSelector;

  frc::Joystick m_keyboard{0};

  bool m_transitionedFromAuto;  ///< True indicates latest enable was during autonomous

  void ConfigureBindings();
};
