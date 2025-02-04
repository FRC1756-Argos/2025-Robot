/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/go_to_position_command.h"

#include "frc2/command/WaitCommand.h"

GoToPositionCommand::GoToPositionCommand(ElevatorSubsystem* elevatorSubsystem, position position)
    : m_pElevatorSubsystem{elevatorSubsystem}, m_position{position} {
  AddRequirements({m_pElevatorSubsystem});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void GoToPositionCommand::Initialize() {
  m_pElevatorSubsystem->SetElevatorManualOverride(false);
  auto currentArmPosistion = m_pElevatorSubsystem->GetArmAngle();
  if ((currentArmPosistion < 90_deg && m_position.arm_angle > 90_deg) ||
      (currentArmPosistion > 90_deg && m_position.arm_angle < 90_deg)) {
    m_pElevatorSubsystem->SetWristAngle(0_deg);
  }
  m_pElevatorSubsystem->ElevatorMoveToHeight(m_position.elevator_hight);
  m_pElevatorSubsystem->ArmMoveToAngle(m_position.arm_angle);
  frc2::WaitCommand(30_ms);
  m_pElevatorSubsystem->SetWristAngle(m_position.wrista_angle);
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
  if (m_pElevatorSubsystem->IsElevatorAtSetPoint() && m_pElevatorSubsystem->IsArmAtSetPoint() &&
      m_pElevatorSubsystem->IsWristAtSetPoint()) {
    return true;
  }
  return false;
}
