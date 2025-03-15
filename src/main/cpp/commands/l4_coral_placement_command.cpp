/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/l4_coral_placement_command.h"

L4CoralPlacementCommand::L4CoralPlacementCommand(ElevatorSubsystem* elevatorSubsystem, IntakeSubsystem* intakeSubsystem)
    : m_pElevatorSubsystem{elevatorSubsystem}, m_pIntakeSubsystem{intakeSubsystem} {
  AddRequirements({m_pElevatorSubsystem, m_pIntakeSubsystem});
}

// Called when the command is initially scheduled.
void L4CoralPlacementCommand::Initialize() {
  auto currentArmPosition = m_pElevatorSubsystem->GetArmAngle();
  if (currentArmPosition < 90_deg) {
    m_pElevatorSubsystem->ArmMoveToAngle(currentArmPosition - 40_deg);
  } else {
    m_pElevatorSubsystem->ArmMoveToAngle(currentArmPosition + 40_deg);
  }

  m_startTime = std::chrono::steady_clock::now();
}

// Called repeatedly when this Command is scheduled to run
void L4CoralPlacementCommand::Execute() {
  if (m_pElevatorSubsystem->GetElevatorManualOverride()) {
    Cancel();
  }

  if (m_pElevatorSubsystem->IsArmAtSetPoint() &&
      (std::chrono::steady_clock::now() - m_startTime) >= std::chrono::milliseconds(100)) {
    m_pElevatorSubsystem->ElevatorMoveToHeight(m_pElevatorSubsystem->GetElevatorHeight() + 2.5_in);
  }
  m_pIntakeSubsystem->Outtake(0.02);
}

// Called once the command ends or is interrupted.
void L4CoralPlacementCommand::End(bool interrupted) {
  if (interrupted) {
    m_pElevatorSubsystem->GoToPosition(m_pElevatorSubsystem->GetPosition());
  }
  m_pIntakeSubsystem->Stop();
}

// Returns true when the command should end.
bool L4CoralPlacementCommand::IsFinished() {
  return m_pElevatorSubsystem->IsAtSetPoint() &&
         (std::chrono::steady_clock::now() - m_startTime) >= std::chrono::milliseconds(500);
}
