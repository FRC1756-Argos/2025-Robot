/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_L4_G_algae.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <units/length.h>

#include "commands/autonomous/auto_utils.h"
#include "commands/drive_choreo.h"

AutonomousL4GAlgae::AutonomousL4GAlgae(ElevatorSubsystem& elevator,
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
    , m_allCommands{
          frc2::SequentialCommandGroup{
              DriveChoreo{m_Swerve, "L4_G_Algae", true, m_armPositionEventCallback, 0},
              frc2::InstantCommand([this]() { m_Swerve.FlipFieldHome(); }, {&m_Swerve}),
              frc2::InstantCommand([this]() { m_Vision.SetLeftAlign(true); }, {&m_Vision}),
              DriveByTimeVisionCommand(m_Swerve, m_Vision, false, 750_ms),
              GoToPositionCommand(&m_Elevator, setpoints::levelFourLeft),
              frc2::WaitCommand(100_ms),
              L4CoralPlacementCommand(&m_Elevator, &m_Intake),
              frc2::ParallelRaceGroup{frc2::WaitCommand(150_ms), GoToPositionCommand(&m_Elevator, setpoints::stow)},
              DriveChoreo{m_Swerve, "L4_G_Algae", false, m_armPositionEventCallback, 2},
              GoToPositionCommand(&m_Elevator, algae::algaePrepNetLeft, false),
              GoToPositionCommand(&m_Elevator, algae::algaeNetRight, false),
              frc2::InstantCommand([this]() { m_Intake.OuttakeAlgae(1.0); }, {&m_Intake}),
              frc2::WaitCommand(500_ms),
              frc2::ParallelRaceGroup{frc2::WaitCommand(150_ms), GoToPositionCommand(&m_Elevator, setpoints::stow)},
              DriveChoreo{m_Swerve, "L4_G_Algae", false, m_armPositionEventCallback, 3},
              GoToPositionCommand(&m_Elevator, algae::algaePrepNetRight, false),
              GoToPositionCommand(&m_Elevator, algae::algaeNetLeft, false),
              frc2::InstantCommand([this]() { m_Intake.OuttakeAlgae(1.0); }, {&m_Intake}),
              frc2::WaitCommand(500_ms),
              frc2::ParallelRaceGroup{frc2::WaitCommand(150_ms), GoToPositionCommand(&m_Elevator, setpoints::stow)}},
          DriveChoreo{m_Swerve, "L4_G_Algae", false, m_armPositionEventCallback, 4}} {}

// Called when the command is initially scheduled.
void AutonomousL4GAlgae::Initialize() {
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousL4GAlgae::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousL4GAlgae::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousL4GAlgae::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousL4GAlgae::GetName() const {
  return "11. L4 G + Algae";
}

frc2::Command* AutonomousL4GAlgae::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
