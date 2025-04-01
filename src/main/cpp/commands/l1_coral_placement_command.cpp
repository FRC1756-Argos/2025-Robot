/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/l1_coral_placement_command.h"

L1CoralPlacementCommand::L1CoralPlacementCommand(ElevatorSubsystem* elevatorSubsystem, IntakeSubsystem* intakeSubsystem)
    : m_pElevatorSubsystem{elevatorSubsystem}, m_pIntakeSubsystem{intakeSubsystem} {
  AddRequirements({m_pElevatorSubsystem, m_pIntakeSubsystem});
}

// Called when the command is initially scheduled.
void L1CoralPlacementCommand::Initialize() {
  m_pIntakeSubsystem->Outtake(0.15);
  m_startTime = std::chrono::steady_clock::now();
}

// Called repeatedly when this Command is scheduled to run
void L1CoralPlacementCommand::Execute() {
  if (m_pElevatorSubsystem->GetElevatorManualOverride()) {
    Cancel();
  }
}

// Called once the command ends or is interrupted.
void L1CoralPlacementCommand::End(bool interrupted) {
  m_pIntakeSubsystem->Stop();
}

// Returns true when the command should end.
bool L1CoralPlacementCommand::IsFinished() {
  return (std::chrono::steady_clock::now() - m_startTime) >= std::chrono::milliseconds(1000);
}
