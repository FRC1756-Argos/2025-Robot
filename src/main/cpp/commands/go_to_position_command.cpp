/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/go_to_position_command.h"

#include "frc2/command/WaitCommand.h"

GoToPositionCommand::GoToPositionCommand(ElevatorSubsystem* elevatorSubsystem, Position position)
    : m_pElevatorSubsystem{elevatorSubsystem}, m_position{position} {
  AddRequirements({m_pElevatorSubsystem});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void GoToPositionCommand::Initialize() {
  auto currentArmPosition = m_pElevatorSubsystem->GetPosition();
  /// @todo make sequential command
  if ((currentArmPosition.arm_angle < 90_deg && m_position.arm_angle > 90_deg) ||
      (currentArmPosition.arm_angle > 90_deg && m_position.arm_angle < 90_deg)) {
    m_pElevatorSubsystem->SetWristAngle(0_deg);
  }
  m_pElevatorSubsystem->ElevatorMoveToHeight(m_position.elevator_height);
  m_pElevatorSubsystem->ArmMoveToAngle(m_position.arm_angle);
  frc2::WaitCommand(30_ms);  /// @todo remove this. Initialize should not block execution
  m_pElevatorSubsystem->SetWristAngle(m_position.wrist_angle);
}

// Called repeatedly when this Command is scheduled to run
void GoToPositionCommand::Execute() {
  if (m_pElevatorSubsystem->GetElevatorManualOverride()) {
    Cancel();
  }
}

// Called once the command ends or is interrupted.
void GoToPositionCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool GoToPositionCommand::IsFinished() {
  return m_pElevatorSubsystem->IsAtSetPoint();
}
