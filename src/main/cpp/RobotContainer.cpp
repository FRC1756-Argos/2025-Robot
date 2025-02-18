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

#include "commands/go_to_position_command.h"
#include "commands/middle_coral_placement_command.h"
#include "constants/position.h"

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
    , m_climberSubSystem(m_instance)
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
  auto fieldHome = m_controllers.DriverController().TriggerDebounced(argos_lib::XboxController::Button::kBack);

  auto intakeLeftTrigger = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kBumperLeft);
  auto intakeRightTrigger =
      m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kBumperRight);
  auto placeRightTrigger =
      m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kRightTrigger);
  auto placeLeftTrigger = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kLeftTrigger);
  auto outtakeTrigger = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kA);

  //auto placeCoral = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kRightTrigger);

  auto climberupTrigger = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kDown);
  auto climberdownTrigger =
      m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kLeftTrigger);

  // Swap controllers config
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});

  auto fireTrigger = m_controllers.OperatorController().TriggerDebounced(argos_lib::XboxController::Button::kStart) &&
                     !m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBack);

  auto elevatorLiftManualInput = (frc2::Trigger{[this]() {
    return std::abs(m_controllers.OperatorController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand)) > 0.2;
  }});
  auto armManualInputLeft = (frc2::Trigger{[this]() {
    return std::abs(m_controllers.OperatorController().GetTriggerAxis(
               argos_lib::XboxController::JoystickHand::kLeftHand)) > 0.2;
  }});
  auto armManualInputRight = (frc2::Trigger{[this]() {
    return std::abs(m_controllers.OperatorController().GetTriggerAxis(
               argos_lib::XboxController::JoystickHand::kRightHand)) > 0.2;
  }});

  auto algaeMode = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBack);
  auto intakeManual = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBumperRight);
  //auto setLeft = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBumperLeft);
  auto goToCoralStation = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBumperLeft);

  auto goToL1 = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kX);
  auto goToL2 = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kA);
  auto goToL3 = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kB);
  auto goToL4 = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kY);
  auto goToStow = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kRight);
  //auto goToSideStow = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kUp);

  auto elevatorWristManualInput = (frc2::Trigger{[this]() {
    return std::abs(m_controllers.OperatorController().GetY(argos_lib::XboxController::JoystickHand::kRightHand)) > 0.2;
  }});

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

  outtakeTrigger.OnTrue(frc2::InstantCommand([this]() { m_intakeSubSystem.Outtake(); }, {&m_intakeSubSystem}).ToPtr());

  (!intakeLeftTrigger && !intakeRightTrigger && !intakeManual && !outtakeTrigger)
      .OnTrue(frc2::InstantCommand([this]() { m_intakeSubSystem.Stop(); }, {&m_intakeSubSystem}).ToPtr());

  (climberupTrigger || climberdownTrigger)
      .OnTrue(frc2::InstantCommand([this]() { m_climberSubSystem.SetClimberManualOverride(true); }, {}).ToPtr());
  m_climberSubSystem.SetDefaultCommand(frc2::RunCommand(
                                           [this] {
                                             double climberupSpeed =
                                                 0.2 * m_controllers.DriverController().GetTriggerAxis(
                                                           argos_lib::XboxController::JoystickHand::kRightHand);
                                             double climberdownSpeed =
                                                 0.2 * m_controllers.DriverController().GetTriggerAxis(
                                                           argos_lib::XboxController::JoystickHand::kLeftHand);
                                             m_climberSubSystem.Move(climberupSpeed - climberdownSpeed);
                                           },
                                           {&m_climberSubSystem})
                                           .ToPtr());

  (elevatorLiftManualInput || elevatorWristManualInput || armManualInputLeft || armManualInputRight)
      .OnTrue(frc2::InstantCommand([this]() { m_elevatorSubSystem.SetElevatorManualOverride(true); }, {}).ToPtr());
  m_elevatorSubSystem.SetDefaultCommand(
      frc2::RunCommand(
          [this] {
            double elevatorSpeed =
                -0.2 * m_controllers.OperatorController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand);
            m_elevatorSubSystem.ElevatorMove(elevatorSpeed);
            double wristSpeed =
                -0.2 * m_controllers.OperatorController().GetY(argos_lib::XboxController::JoystickHand::kRightHand);
            m_elevatorSubSystem.Rotate(wristSpeed);
            double armSpeed = -0.2 * m_controllers.OperatorController().GetTriggerAxis(
                                         argos_lib::XboxController::JoystickHand::kLeftHand) +
                              0.2 * m_controllers.OperatorController().GetTriggerAxis(
                                        argos_lib::XboxController::JoystickHand::kRightHand);
            m_elevatorSubSystem.Pivot(armSpeed);
          },
          {&m_elevatorSubSystem})
          .ToPtr());

  goToStow.OnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::stow).ToPtr());
  (intakeManual).OnTrue(frc2::InstantCommand([this]() { m_intakeSubSystem.Intake(); }, {&m_intakeSubSystem}).ToPtr());

  (placeLeftTrigger)
      .ToggleOnFalse(MiddleCoralPlacementCommand(&m_elevatorSubSystem, &m_intakeSubSystem)
                         .ToPtr()
                         .AndThen(GoToPositionCommand(&m_elevatorSubSystem, setpoints::stow).ToPtr()));

  //L1 Logic
  (!algaeMode && placeLeftTrigger && goToL1)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::levelOneLeft).ToPtr());

  //(placeLeftTrigger).ToggleOnFalse(GoToPositionCommand(&m_elevatorSubSystem, setpoints::stow).ToPtr());

  //L2 Logic
  (!algaeMode && placeLeftTrigger && goToL2)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::levelTwoLeft).ToPtr());

  //L3 Logic
  (!algaeMode && placeLeftTrigger && goToL3)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::levelThreeLeft).ToPtr());

  //L4 Logic
  (!algaeMode && placeLeftTrigger && goToL4)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::levelFourCenter)
                  .ToPtr()
                  .AndThen(GoToPositionCommand(&m_elevatorSubSystem, setpoints::levelFourLeft).ToPtr()));

  //Coral Station
  (!algaeMode && intakeLeftTrigger && goToCoralStation)
      .ToggleOnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::coralStationLeft).ToPtr());
  (!algaeMode && intakeLeftTrigger && !goToCoralStation)
      .ToggleOnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::floorIntakeLeft).ToPtr());
  (!algaeMode && intakeLeftTrigger)
      .OnTrue(frc2::InstantCommand([this]() { m_intakeSubSystem.Intake(); }, {&m_intakeSubSystem}).ToPtr());
  (!algaeMode && intakeLeftTrigger)
      .ToggleOnFalse(GoToPositionCommand(&m_elevatorSubSystem, internal::highLeft).ToPtr());

  (placeRightTrigger)
      .ToggleOnFalse(MiddleCoralPlacementCommand(&m_elevatorSubSystem, &m_intakeSubSystem)
                         .ToPtr()
                         .AndThen(GoToPositionCommand(&m_elevatorSubSystem, setpoints::stow).ToPtr()));

  //L1 Logic
  (!algaeMode && placeRightTrigger && goToL1)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::levelOneRight).ToPtr());

  //L2 Logic
  (!algaeMode && placeRightTrigger && goToL2)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::levelTwoRight).ToPtr());

  //L3 Logic
  (!algaeMode && placeRightTrigger && goToL3)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::levelThreeRight).ToPtr());

  //L4 Logic
  (!algaeMode && placeRightTrigger && goToL4)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::levelFourCenter)
                  .ToPtr()
                  .AndThen(GoToPositionCommand(&m_elevatorSubSystem, setpoints::levelFourRight).ToPtr()));

  //Coral Station
  (!algaeMode && intakeRightTrigger && goToCoralStation)
      .ToggleOnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::coralStationRight).ToPtr());
  (!algaeMode && intakeRightTrigger && !goToCoralStation)
      .ToggleOnTrue(GoToPositionCommand(&m_elevatorSubSystem, setpoints::floorIntakeRight).ToPtr());
  (!algaeMode && intakeRightTrigger)
      .OnTrue(frc2::InstantCommand([this]() { m_intakeSubSystem.Intake(); }, {&m_intakeSubSystem}).ToPtr());
  (!algaeMode && intakeRightTrigger)
      .ToggleOnFalse(GoToPositionCommand(&m_elevatorSubSystem, internal::highRight).ToPtr());

  // Algae Controls
  (algaeMode && intakeLeftTrigger && goToL1)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, algae::algaeProcessorLeft).ToPtr());
  (algaeMode && intakeLeftTrigger && goToL2)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, algae::algaeLowLeft).ToPtr());
  (algaeMode && intakeLeftTrigger && goToL3)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, algae::algaeHighLeft).ToPtr());
  (algaeMode && intakeLeftTrigger && goToL4)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, algae::algaeNetLeft).ToPtr());
  (algaeMode && intakeRightTrigger && goToL1)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, algae::algaeProcessorRight).ToPtr());
  (algaeMode && intakeRightTrigger && goToL2)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, algae::algaeLowRight).ToPtr());
  (algaeMode && intakeRightTrigger && goToL3)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, algae::algaeHighRight).ToPtr());
  (algaeMode && intakeRightTrigger && goToL4)
      .OnTrue(GoToPositionCommand(&m_elevatorSubSystem, algae::algaeNetRight).ToPtr());

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
