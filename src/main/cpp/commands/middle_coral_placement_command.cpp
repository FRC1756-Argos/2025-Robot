/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/middle_coral_placement_command.h"

#include "constants/position.h"

MiddleCoralPlacementCommand::MiddleCoralPlacementCommand(ElevatorSubsystem* elevatorSubsystem,
                                                         IntakeSubsystem* intakeSubsystem)
    : m_pElevatorSubsystem{elevatorSubsystem}, m_pIntakeSubsystem{intakeSubsystem} {
  AddRequirements({m_pElevatorSubsystem, m_pIntakeSubsystem});
}

// Called when the command is initially scheduled.
void MiddleCoralPlacementCommand::Initialize() {
  auto currentArmPosition = m_pElevatorSubsystem->GetArmAngle();
  if (currentArmPosition < 90_deg) {
    m_pElevatorSubsystem->ArmMoveToAngle(setpoints::levelThreeLeft.arm_angle - 28_deg);
  } else {
    m_pElevatorSubsystem->ArmMoveToAngle(setpoints::levelThreeRight.arm_angle + 28_deg);
  }

  m_startTime = std::chrono::steady_clock::now();
}

// Called repeatedly when this Command is scheduled to run
void MiddleCoralPlacementCommand::Execute() {
  if (m_pElevatorSubsystem->GetElevatorManualOverride()) {
    Cancel();
  }

  if (m_pElevatorSubsystem->IsArmAtSetPoint() &&
      (std::chrono::steady_clock::now() - m_startTime) >= std::chrono::milliseconds(300)) {
    m_pElevatorSubsystem->ElevatorMoveToHeight(m_pElevatorSubsystem->GetElevatorHeight() - 3_in);
    //m_pIntakeSubsystem->Outtake(-0.4);
  }
}

// Called once the command ends or is interrupted.
void MiddleCoralPlacementCommand::End(bool interrupted) {
  if (interrupted) {
    m_pElevatorSubsystem->GoToPosition(m_pElevatorSubsystem->GetPosition());
  }
  m_pIntakeSubsystem->Stop();
}

// Returns true when the command should end.
bool MiddleCoralPlacementCommand::IsFinished() {
  return m_pElevatorSubsystem->IsAtSetPoint() &&
         (std::chrono::steady_clock::now() - m_startTime) >= std::chrono::milliseconds(500);
}
