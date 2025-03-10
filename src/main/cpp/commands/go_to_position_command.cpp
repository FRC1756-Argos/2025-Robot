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

GoToPositionCommand::GoToPositionCommand(ElevatorSubsystem* elevatorSubsystem, Position position, bool coralMode)
    : m_pElevatorSubsystem{elevatorSubsystem}, m_position{position}, m_coralMode{coralMode} {
  AddRequirements(m_pElevatorSubsystem);
}

// Called when the command is initially scheduled.
void GoToPositionCommand::Initialize() {
  m_pElevatorSubsystem->ArmMoveToAngle(GetSafeArmTarget(m_position.arm_angle));
  m_pElevatorSubsystem->ElevatorMoveToHeight(m_position.elevator_height);
  m_startTime = std::chrono::high_resolution_clock::now();
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

  // Keep arm inside robot for most of travel so we don't hit the reef
  m_pElevatorSubsystem->ArmMoveToAngle(GetSafeArmTarget(m_position.arm_angle));
}

// Called once the command ends or is interrupted.
void GoToPositionCommand::End(bool interrupted) {
  if (interrupted) {
    m_pElevatorSubsystem->GoToPosition(m_pElevatorSubsystem->GetPosition());
  }
}

// Returns true when the command should end.
bool GoToPositionCommand::IsFinished() {
  auto currentTime = std::chrono::high_resolution_clock::now();
  return (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - m_startTime) >=
          std::chrono::milliseconds(1500)) ||
         (m_pElevatorSubsystem->GetPosition().AlmostEqual(m_position));
}

units::degree_t GoToPositionCommand::GetSafeArmTarget(units::degree_t target) {
  if (m_coralMode && units::math::abs(m_pElevatorSubsystem->GetElevatorHeight() - m_position.elevator_height) > 5_in) {
    return std::clamp<units::degree_t>(
        target, measure_up::elevator::arm::internalMinAngle, measure_up::elevator::arm::internalMaxAngle);
  }
  return target;
}
