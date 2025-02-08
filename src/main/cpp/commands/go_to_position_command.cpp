/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/go_to_position_command.h"

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <units/math.h>

#include <algorithm>
#include <vector>

GoToPositionCommand::GoToPositionCommand(ElevatorSubsystem* elevatorSubsystem, Position position)
    : m_pElevatorSubsystem{elevatorSubsystem}, m_position{position}, m_waypoints{} {
  AddRequirements(m_pElevatorSubsystem);
}

// Called when the command is initially scheduled.
void GoToPositionCommand::Initialize() {
  auto currentPosition = m_pElevatorSubsystem->GetPosition();
  const bool armAngleDecreasing = (m_position.arm_angle - currentPosition.arm_angle) < 0_deg;

  // Generate arm waypoints (no elevator) to avoid hitting floor or elevator
  m_waypoints.clear();
  if ((currentPosition.arm_angle < internal::lowLeft.arm_angle &&
       m_position.arm_angle >= internal::lowLeft.arm_angle) ||
      (currentPosition.arm_angle >= internal::lowLeft.arm_angle &&
       m_position.arm_angle < internal::lowLeft.arm_angle)) {
    m_waypoints.push_back(internal::lowLeft);
  }
  if ((currentPosition.arm_angle < internal::highLeft.arm_angle &&
       m_position.arm_angle >= internal::highLeft.arm_angle) ||
      (currentPosition.arm_angle >= internal::highLeft.arm_angle &&
       m_position.arm_angle < internal::highLeft.arm_angle)) {
    m_waypoints.push_back(internal::highLeft);
  }
  if ((currentPosition.arm_angle < internal::highRight.arm_angle &&
       m_position.arm_angle >= internal::highRight.arm_angle) ||
      (currentPosition.arm_angle >= internal::highRight.arm_angle &&
       m_position.arm_angle < internal::highRight.arm_angle)) {
    m_waypoints.push_back(internal::highRight);
  }
  if ((currentPosition.arm_angle < internal::lowRight.arm_angle &&
       m_position.arm_angle >= internal::lowRight.arm_angle) ||
      (currentPosition.arm_angle >= internal::lowRight.arm_angle &&
       m_position.arm_angle < internal::lowRight.arm_angle)) {
    m_waypoints.push_back(internal::lowRight);
  }

  // If arm is going from larger angle to smaller angle, waypoints should also be in decreasing angle order
  if (armAngleDecreasing) {
    std::reverse(m_waypoints.begin(), m_waypoints.end());
  }

  // Linear interpolate elevator heights for waypoints
  auto armAngleRange = m_position.arm_angle - currentPosition.arm_angle;
  auto elevatorHeightRange = m_position.elevator_height - currentPosition.elevator_height;
  for (auto& waypoint : m_waypoints) {
    auto armAnglePercentageComplete = (waypoint.arm_angle - currentPosition.arm_angle) / armAngleRange;
    waypoint.elevator_height = currentPosition.elevator_height + (elevatorHeightRange * armAnglePercentageComplete);
  }

  m_waypoints.push_back(m_position);

  m_pElevatorSubsystem->GoToPosition(m_waypoints.front());
}

// Called repeatedly when this Command is scheduled to run
void GoToPositionCommand::Execute() {
  if (m_pElevatorSubsystem->GetElevatorManualOverride()) {
    Cancel();
  }
  if (m_pElevatorSubsystem->IsAtSetPoint() && m_waypoints.front().AlmostEqual(m_pElevatorSubsystem->GetSetpoint())) {
    m_waypoints.erase(m_waypoints.begin());
    m_pElevatorSubsystem->GoToPosition(m_waypoints.front());
  }
}

// Called once the command ends or is interrupted.
void GoToPositionCommand::End(bool interrupted) {
  if (interrupted) {
    m_pElevatorSubsystem->GoToPosition(m_pElevatorSubsystem->GetPosition());
  }
  m_waypoints.clear();
}

// Returns true when the command should end.
bool GoToPositionCommand::IsFinished() {
  return m_waypoints.empty();
}
