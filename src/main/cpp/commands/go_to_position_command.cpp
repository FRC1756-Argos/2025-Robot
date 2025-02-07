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
  auto currentPosition = m_pElevatorSubsystem->GetPosition();
  if ((currentPosition.arm_angle < internal::lowRight.arm_angle &&
       m_position.arm_angle >= internal::lowRight.arm_angle && m_position.arm_angle < internal::highRight.arm_angle) ||
      (currentPosition.arm_angle >= internal::lowRight.arm_angle &&
       m_position.arm_angle < internal::lowRight.arm_angle &&
       currentPosition.arm_angle < internal::highRight.arm_angle)) {
    // goto 5
  }
  if ((currentPosition.arm_angle > internal::lowLeft.arm_angle && m_position.arm_angle <= internal::lowLeft.arm_angle &&
       m_position.arm_angle > internal::highLeft.arm_angle) ||
      (currentPosition.arm_angle <= internal::lowLeft.arm_angle && m_position.arm_angle > internal::lowLeft.arm_angle &&
       currentPosition.arm_angle > internal::highLeft.arm_angle)) {
    // goto 175
  }
  if ((currentPosition.arm_angle >= internal::lowRight.arm_angle &&
       m_position.arm_angle > internal::highRight.arm_angle && m_position.arm_angle < 90_deg) ||
      (currentPosition.arm_angle > internal::highRight.arm_angle &&
       m_position.arm_angle <= internal::highRight.arm_angle && currentPosition.arm_angle < 90_deg)) {
    // goto 5
  }
  if ((currentPosition.arm_angle < internal::lowLeft.arm_angle && m_position.arm_angle <= internal::lowLeft.arm_angle &&
       m_position.arm_angle >= 90_deg) ||
      (currentPosition.arm_angle >= internal::highLeft.arm_angle &&
       m_position.arm_angle > internal::lowLeft.arm_angle && currentPosition.arm_angle >= 90_deg)) {
    // goto 175
  }
  /// @todo make sequential command
  if ((currentPosition.arm_angle < 90_deg && m_position.arm_angle > 90_deg) ||
      (currentPosition.arm_angle > 90_deg && m_position.arm_angle < 90_deg)) {
    m_pElevatorSubsystem->SetWristAngle(0_deg);
  }
  m_pElevatorSubsystem->ElevatorMoveToHeight(m_position.elevator_height);
  m_pElevatorSubsystem->ArmMoveToAngle(m_position.arm_angle);

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
