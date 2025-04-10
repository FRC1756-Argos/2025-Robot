/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_L1_IJ.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <units/length.h>

#include "commands/autonomous/auto_utils.h"
#include "commands/drive_choreo.h"
#include "commands/go_to_position_command.h"
#include "commands/l1_coral_placement_command.h"

AutonomousL1IJ::AutonomousL1IJ(ElevatorSubsystem& elevator,
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
    , m_allCommands{frc2::SequentialCommandGroup{frc2::InstantCommand([this]() { m_Vision.Disable(); }, {&m_Vision}),
                                                 DriveChoreo{m_Swerve, "L1_IJ", true, m_armPositionEventCallback},
                                                 L1CoralPlacementCommand(&m_Elevator, &m_Intake),
                                                 GoToPositionCommand(&m_Elevator, setpoints::stow)}} {}

// Called when the command is initially scheduled.
void AutonomousL1IJ::Initialize() {
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousL1IJ::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousL1IJ::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousL1IJ::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousL1IJ::GetName() const {
  return "05. L1 IJ";
}

frc2::Command* AutonomousL1IJ::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
