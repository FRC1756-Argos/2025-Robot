/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "RobotContainer.h"

#include <argos_lib/commands/swap_controllers_command.h>
#include <argos_lib/controller/trigger_composition.h>
#include <argos_lib/general/angle_utils.h>
#include <argos_lib/general/color.h>
#include <argos_lib/general/swerve_utils.h>
#include <frc/DriverStation.h>
#include <frc/RobotState.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/button/Trigger.h>
#include <units/angular_velocity.h>
#include <units/length.h>

// Include GamePiece enum
#include <constants/feature_flags.h>
#include <Constants.h>

#include <cmath>
#include <memory>
#include <optional>

#include "utils/custom_units.h"

RobotContainer::RobotContainer()
    : m_driveSpeedMap(controllerMap::driveSpeed)
    , m_driveRotSpeed(controllerMap::driveRotSpeed)
    , m_instance(argos_lib::GetRobotInstance())
    , m_controllers(address::comp_bot::controllers::driver, address::comp_bot::controllers::secondary)
    , m_swerveDrive(m_instance)
    , m_ledSubSystem(m_instance)
    , m_visionSubSystem(m_instance, &m_swerveDrive)
    , m_elevatorSubSystem(m_instance)
    , m_intakeSubSystem(m_instance)
    , m_autoNothing{m_swerveDrive}
    , m_autoSelector{{&m_autoNothing}, &m_autoNothing}
    , m_transitionedFromAuto{false} {
  // Initialize all of your commands and subsystems here

  AllianceChanged();

  // ================== DEFAULT COMMANDS ===============================
  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        auto deadbandTranslationSpeeds = argos_lib::swerve::CircularInterpolate(
            argos_lib::swerve::TranslationSpeeds{
                -m_controllers.DriverController().GetY(
                    argos_lib::XboxController::JoystickHand::kLeftHand),  // Y axis is negative forward
                -m_controllers.DriverController().GetX(
                    argos_lib::XboxController::JoystickHand::
                        kLeftHand)},  // X axis is positive right, but swerve coordinates are positive left
            m_driveSpeedMap);
        auto deadbandRotSpeed = m_driveRotSpeed(
            -m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kRightHand));

        auto rotateSpeed = deadbandRotSpeed;

        if (frc::DriverStation::IsTeleop() &&
            (m_swerveDrive.GetManualOverride() || deadbandTranslationSpeeds.forwardSpeedPct != 0 ||
             deadbandTranslationSpeeds.leftSpeedPct != 0 || rotateSpeed != 0)) {
          m_visionSubSystem.SetEnableStaticRotation(false);
          m_swerveDrive.SwerveDrive(
              deadbandTranslationSpeeds.forwardSpeedPct,
              deadbandTranslationSpeeds.leftSpeedPct,
              rotateSpeed);  // X axis is positive right (CW), but swerve coordinates are positive left (CCW)
        }
        // DEBUG STUFF
        if constexpr (feature_flags::nt_debugging) {
          frc::SmartDashboard::PutBoolean("(DRIVER) Static Enable", m_visionSubSystem.IsStaticRotationEnabled());
          frc::SmartDashboard::PutNumber(
              "(DRIVER) Joystick Left Y",
              m_controllers.DriverController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand));
          frc::SmartDashboard::PutNumber(
              "(DRIVER) Joystick Left X",
              m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kLeftHand));
          frc::SmartDashboard::PutNumber(
              "(DRIVER) Joystick Right X",
              m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kRightHand));
        }
      },
      {&m_swerveDrive}));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  /* ———————————————————————— CONFIGURE DEBOUNCING ——————————————————————— */

  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kY, {1500_ms, 0_ms});

  /* —————————————————————————————— TRIGGERS ————————————————————————————— */

  auto robotEnableTrigger = (frc2::Trigger{[this]() { return frc::DriverStation::IsEnabled(); }});

  // DRIVE TRIGGERS
  auto fieldHome = m_controllers.DriverController().TriggerDebounced(argos_lib::XboxController::Button::kY);

  auto intakeTrigger = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kBumperRight);
  auto outtakeTrigger = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kBumperLeft);

  // Swap controllers config
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});

  auto fireTrigger = m_controllers.OperatorController().TriggerDebounced(argos_lib::XboxController::Button::kStart) &&
                     !m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBack);

  // SWAP CONTROLLER TRIGGERS
  frc2::Trigger driverTriggerSwapCombo = m_controllers.DriverController().TriggerDebounced(
      {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});
  frc2::Trigger operatorTriggerSwapCombo = m_controllers.OperatorController().TriggerDebounced(
      {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});

  /* ————————————————————————— TRIGGER ACTIVATION ———————————————————————— */

  robotEnableTrigger.OnTrue(frc2::InstantCommand([this]() {
                              if (frc::DriverStation::IsAutonomous()) {
                                m_transitionedFromAuto = true;
                              } else {
                                m_transitionedFromAuto = false;
                              }
                            }).ToPtr());

  // DRIVE TRIGGER ACTIVATION
  fieldHome.OnTrue(frc2::InstantCommand([this]() { m_swerveDrive.FieldHome(); }, {&m_swerveDrive}).ToPtr());
  intakeTrigger.OnTrue(frc2::InstantCommand([this]() { m_intakeSubSystem.Intake(); }, {&m_intakeSubSystem}).ToPtr());
  outtakeTrigger.OnTrue(frc2::InstantCommand([this]() { m_intakeSubSystem.Outtake(); }, {&m_intakeSubSystem}).ToPtr());
  (!intakeTrigger && !outtakeTrigger)
      .OnTrue(frc2::InstantCommand([this]() { m_intakeSubSystem.Stop(); }, {&m_intakeSubSystem}).ToPtr());

  // SWAP CONTROLLERS TRIGGER ACTIVATION
  (driverTriggerSwapCombo || operatorTriggerSwapCombo)
      .WhileTrue(argos_lib::SwapControllersCommand(&m_controllers).ToPtr());
}

void RobotContainer::Disable() {
  m_ledSubSystem.Disable();
  m_swerveDrive.Disable();
  m_intakeSubSystem.Disable();
}

void RobotContainer::Enable() {
  m_ledSubSystem.Enable();
}

void RobotContainer::AllianceChanged() {
  // If disabled, set alliance colors
  m_ledSubSystem.SetAllGroupsAllianceColor(true, true);
  m_ledSubSystem.SetDisableAnimation([this]() { m_ledSubSystem.SetAllGroupsAllianceColor(false, false); });
}

void RobotContainer::SetLedsConnectedBrightness(bool connected) {
  m_ledSubSystem.SetLedsConnectedBrightness(connected);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoSelector.GetSelectedCommand();
}
