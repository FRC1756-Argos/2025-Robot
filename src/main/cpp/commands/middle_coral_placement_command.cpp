/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/middle_coral_placement_command.h"

MiddleCoralPlacementCommand::MiddleCoralPlacementCommand(ElevatorSubsystem* elevatorSubsystem)
    : m_pElevatorSubsystem{elevatorSubsystem} {
  AddRequirements(m_pElevatorSubsystem);
}

// Called when the command is initially scheduled.
void MiddleCoralPlacementCommand::Initialize() {
  auto currentArmPosition = m_pElevatorSubsystem->GetArmAngle();
  if (currentArmPosition < 90_deg) {
    m_pElevatorSubsystem->ArmMoveToAngle(currentArmPosition - 3_deg);
  } else {
    m_pElevatorSubsystem->ArmMoveToAngle(currentArmPosition + 3_deg);
  }
  m_pElevatorSubsystem->ElevatorMoveToHeight(m_pElevatorSubsystem->GetElevatorHeight() - 5_in);
}

// Called repeatedly when this Command is scheduled to run
void MiddleCoralPlacementCommand::Execute() {
  if (m_pElevatorSubsystem->GetElevatorManualOverride()) {
    Cancel();
  }
}

// Called once the command ends or is interrupted.
void MiddleCoralPlacementCommand::End(bool interrupted) {
  if (interrupted) {
    m_pElevatorSubsystem->GoToPosition(m_pElevatorSubsystem->GetPosition());
  }
}

// Returns true when the command should end.
bool MiddleCoralPlacementCommand::IsFinished() {
  return m_pElevatorSubsystem->IsAtSetPoint();
}
