/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include "constants/position.h"
#include "subsystems/elevator_subsystem.h"
#include "subsystems/intake_subsystem.h"

namespace auto_utils {

  void SetAutoArmPosition(ArmPosition position, ElevatorSubsystem* elevator, IntakeSubsystem* intake);
  ArmPosition stringToPosition(const std::string str);
  std::string toString(ArmPosition position);
  std::vector<ArmPosition> getPositions();
  std::vector<std::string> getPositionStrings();

}  // namespace auto_utils
