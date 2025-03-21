/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_L4_EDC.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <units/length.h>

#include "commands/autonomous/auto_utils.h"
#include "commands/drive_choreo.h"
#include "commands/go_to_position_command.h"

/// @note Splits 1, 4, and 7 are follow through so the robot doesn't stop when transitioning to vision alignment

AutonomousL4EDC::AutonomousL4EDC(ElevatorSubsystem& elevator,
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
          DriveChoreo{m_Swerve, "L4_EDC", true, m_armPositionEventCallback, 0},
          frc2::InstantCommand([this]() { m_Vision.SetLeftAlign(true); }, {&m_Vision}),
          DriveByTimeVisionCommand(m_Swerve, m_Vision, false, 1250_ms),
          GoToPositionCommand(&m_Elevator, setpoints::levelFourLeft),
          frc2::WaitCommand(200_ms),
          L4CoralPlacementCommand(&m_Elevator, &m_Intake),
          frc2::ParallelRaceGroup{frc2::WaitCommand(150_ms), GoToPositionCommand(&m_Elevator, setpoints::stow)},
          DriveChoreo{m_Swerve, "L4_EDC", false, m_armPositionEventCallback, 2},
          GoToPositionCommand(&m_Elevator, setpoints::coralStationRight),
          frc2::WaitCommand(200_ms),
          DriveChoreo{m_Swerve, "L4_EDC", false, m_armPositionEventCallback, 3},
          frc2::InstantCommand([this]() { m_Vision.SetLeftAlign(true); }, {&m_Vision}),  /// @todo This should be right
          DriveByTimeVisionCommand(m_Swerve, m_Vision, false, 950_ms),
          GoToPositionCommand(&m_Elevator, setpoints::levelFourLeft),
          frc2::WaitCommand(300_ms),
          L4CoralPlacementCommand(&m_Elevator, &m_Intake),
          frc2::ParallelRaceGroup{frc2::WaitCommand(150_ms), GoToPositionCommand(&m_Elevator, setpoints::stow)},
          DriveChoreo{m_Swerve, "L4_EDC", false, m_armPositionEventCallback, 5},
          GoToPositionCommand(&m_Elevator, setpoints::coralStationRight),
          frc2::WaitCommand(200_ms),
          DriveChoreo{m_Swerve, "L4_EDC", false, m_armPositionEventCallback, 6},
          frc2::InstantCommand([this]() { m_Vision.SetRightAlign(true); }, {&m_Vision}),  /// @todo This should be left
          DriveByTimeVisionCommand(m_Swerve, m_Vision, false, 950_ms),
          GoToPositionCommand(&m_Elevator, setpoints::levelFourLeft),
          frc2::WaitCommand(300_ms),
          L4CoralPlacementCommand(&m_Elevator, &m_Intake),
          GoToPositionCommand(&m_Elevator, setpoints::stow)}} {}

// Called when the command is initially scheduled.
void AutonomousL4EDC::Initialize() {
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousL4EDC::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousL4EDC::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousL4EDC::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousL4EDC::GetName() const {
  return "08. L4 EDC";
}

frc2::Command* AutonomousL4EDC::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
