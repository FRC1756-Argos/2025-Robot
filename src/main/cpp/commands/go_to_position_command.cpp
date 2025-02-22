/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/go_to_position_command.h"

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <units/math.h>
#include <wpi/raw_ostream.h>

#include <algorithm>
#include <vector>

GoToPositionCommand::GoToPositionCommand(ElevatorSubsystem* elevatorSubsystem, Position position)
    : m_pElevatorSubsystem{elevatorSubsystem}, m_position{position} {
  AddRequirements(m_pElevatorSubsystem);
}

// Called when the command is initially scheduled.
void GoToPositionCommand::Initialize() {
  m_pElevatorSubsystem->ArmMoveToAngle(m_position.arm_angle);
  m_pElevatorSubsystem->ElevatorMoveToHeight(m_position.elevator_height);
}

// Called repeatedly when this Command is scheduled to run
void GoToPositionCommand::Execute() {
  if (m_pElevatorSubsystem->GetElevatorManualOverride()) {
    Cancel();
  }

  auto currentArmAngle = m_pElevatorSubsystem->GetArmAngle();
  bool isCurrentlyInsideLeftBound = false;
  bool isFinalPositionRight = false;
  bool isCurrentlyInsideRightBound = false;
  bool isFinalPositionLeft = false;
  bool isAboveMin = false;

  if (currentArmAngle > internal::lowLeft.arm_angle && currentArmAngle < internal::highLeft.arm_angle) {
    isCurrentlyInsideLeftBound = true;
  }
  if (m_position.arm_angle > internal::highRight.arm_angle) {
    isFinalPositionRight = true;
  }
  if (currentArmAngle < internal::lowRight.arm_angle && currentArmAngle > internal::highRight.arm_angle) {
    isCurrentlyInsideRightBound = true;
  }
  if (m_position.arm_angle < internal::highLeft.arm_angle) {
    isFinalPositionLeft = true;
  }
  if (currentArmAngle > internal::lowLeft.arm_angle && currentArmAngle < internal::lowRight.arm_angle) {
    isAboveMin = true;
  }

  if ((isCurrentlyInsideLeftBound && isFinalPositionRight) || (isCurrentlyInsideRightBound && isFinalPositionLeft) ||
      (!isFinalPositionLeft && !isFinalPositionRight && isAboveMin)) {
    m_pElevatorSubsystem->SetWristAngle(0_deg);
  }
  if ((isCurrentlyInsideLeftBound && isFinalPositionLeft) || (isCurrentlyInsideRightBound && isFinalPositionRight)) {
    m_pElevatorSubsystem->SetWristAngle(m_position.wrist_angle);
  }
}

// Called once the command ends or is interrupted.
void GoToPositionCommand::End(bool interrupted) {
  if (interrupted) {
    m_pElevatorSubsystem->GoToPosition(m_pElevatorSubsystem->GetPosition());
  }
}

// Returns true when the command should end.
bool GoToPositionCommand::IsFinished() {
  return m_pElevatorSubsystem->GetPosition().AlmostEqual(m_position);
}
