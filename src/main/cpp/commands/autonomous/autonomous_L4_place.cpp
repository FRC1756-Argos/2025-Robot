/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <units/length.h>

#include "commands/autonomous/auto_utils.h"
#include "commands/autonomous/autonomous_L4_Place.h"
#include "commands/drive_choreo.h"
#include "commands/go_to_position_command.h"

AutonomousL4Place::AutonomousL4Place(ElevatorSubsystem& elevator,
                                     IntakeSubsystem& intake,
                                     SwerveDriveSubsystem& swerve,
                                     VisionSubsystem& vision)
    : m_Elevator{elevator}
    , m_Intake{intake}
    , m_Swerve{swerve}
    , m_Vision{vision}
    , m_armPositionEventCallback{[this](ArmPosition position) {
      auto_utils::SetAutoArmPosition(position, &m_Elevator, &m_Intake);
    }}
    , m_allCommands{frc2::SequentialCommandGroup{
          DriveChoreo{m_Swerve, "L4Place_1", true, m_armPositionEventCallback},
          GoToPositionCommand(&m_Elevator, setpoints::stow),
          frc2::InstantCommand([this]() { m_Vision.SetRightAlign(true); }, {&m_Vision}),
          DriveByTimeVisionCommand(m_Swerve, m_Vision, false, 1000_ms),
          frc2::InstantCommand([this]() { m_Vision.SetRightAlign(false); }, {&m_Vision}),
          GoToPositionCommand(&m_Elevator, setpoints::levelFourRight),
          frc2::WaitCommand(200_ms),
          L4CoralPlacementCommand(&m_Elevator, &m_Intake),
          GoToPositionCommand(&m_Elevator, setpoints::stow),
          DriveChoreo{m_Swerve, "L4Place_2", true, m_armPositionEventCallback},
          GoToPositionCommand(&m_Elevator, setpoints::coralStationLeft),
          frc2::InstantCommand([this]() { m_Intake.Intake(0.15); }, {&m_Intake}),
          frc2::WaitCommand(750_ms),
          GoToPositionCommand(&m_Elevator, setpoints::stow),
          DriveChoreo{m_Swerve, "L4Place_3", true, m_armPositionEventCallback},
          frc2::InstantCommand([this]() { m_Vision.SetLeftAlign(true); }, {&m_Vision}),
          DriveByTimeVisionCommand(m_Swerve, m_Vision, false, 1000_ms),
          frc2::InstantCommand([this]() { m_Vision.SetLeftAlign(false); }, {&m_Vision}),
          GoToPositionCommand(&m_Elevator, setpoints::levelFourLeft),
          frc2::WaitCommand(200_ms),
          L4CoralPlacementCommand(&m_Elevator, &m_Intake),
          GoToPositionCommand(&m_Elevator, setpoints::stow)

          //DriveChoreo{m_Swerve, "L4Place_3", true, m_armPositionEventCallback}
          //frc2::InstantCommand([this]() { m_Intake.Outtake(0.15); }, {&m_Intake}),
          //frc2::WaitCommand(300_ms),
          //GoToPositionCommand(&m_Elevator, setpoints::stow)
      }} {}

// Called when the command is initially scheduled.
void AutonomousL4Place::Initialize() {
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousL4Place::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousL4Place::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousL4Place::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousL4Place::GetName() const {
  return "6. L4 Place";
}

frc2::Command* AutonomousL4Place::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
