/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_L1_J_L4_L.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <units/length.h>

#include "commands/autonomous/auto_utils.h"
#include "commands/drive_choreo.h"
#include "commands/go_to_position_command.h"
#include "commands/l1_coral_placement_command.h"

AutonomousL1JL4L::AutonomousL1JL4L(ElevatorSubsystem& elevator,
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
          frc2::SequentialCommandGroup{DriveChoreo{m_Swerve, "L4_JKL", true, m_armPositionEventCallback, 0},
                                       frc2::InstantCommand([this]() { m_Vision.SetLeftAlign(true); }, {&m_Vision}),
                                       DriveByTimeVisionCommand(m_Swerve, m_Vision, false, 750_ms),
                                       GoToPositionCommand(&m_Elevator, setpoints::levelOneRight),
                                       //  frc2::WaitCommand(400_ms),
                                       L1CoralPlacementCommand(&m_Elevator, &m_Intake),
                                       DriveChoreo{m_Swerve, "L4_JKL", false, m_armPositionEventCallback, 2},
                                       GoToPositionCommand(&m_Elevator, setpoints::coralStationLeft),
                                       frc2::WaitCommand(500_ms),
                                       DriveChoreo{m_Swerve, "L4_JKL", false, m_armPositionEventCallback, 3},
                                       frc2::InstantCommand([this]() { m_Vision.SetLeftAlign(true); }, {&m_Vision}),
                                       DriveByTimeVisionCommand(m_Swerve, m_Vision, false, 750_ms),
                                       GoToPositionCommand(&m_Elevator, setpoints::levelFourRight),
                                       frc2::WaitCommand(450_ms),
                                       L4CoralPlacementCommand(&m_Elevator, &m_Intake),
                                       GoToPositionCommand(&m_Elevator, setpoints::stow)}} {}

// Called when the command is initially scheduled.
void AutonomousL1JL4L::Initialize() {
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousL1JL4L::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousL1JL4L::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousL1JL4L::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousL1JL4L::GetName() const {
  return "07. L1 J, L4 L";
}

frc2::Command* AutonomousL1JL4L::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
