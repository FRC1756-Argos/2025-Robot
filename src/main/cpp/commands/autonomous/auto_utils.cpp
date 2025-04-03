/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/auto_utils.h"

void auto_utils::SetAutoArmPosition(ArmPosition position, ElevatorSubsystem* elevator, IntakeSubsystem* intake) {
  switch (position) {
    case ArmPosition::STOW:
      elevator->GoToPosition(setpoints::stow);
      intake->Stop();
      break;
    case ArmPosition::LEFT_1:
      elevator->GoToPosition(setpoints::levelOneLeft);
      intake->Stop();
      break;
    case ArmPosition::LEFT_2:
      elevator->GoToPosition(setpoints::levelTwoLeft);
      intake->Stop();
      break;
    case ArmPosition::LEFT_3:
      elevator->GoToPosition(setpoints::levelThreeLeft);
      intake->Stop();
      break;
    case ArmPosition::LEFT_4:
      elevator->GoToPosition(setpoints::levelFourLeft);
      intake->Stop();
      break;
    case ArmPosition::LEFT_STATION:
      elevator->GoToPosition(setpoints::coralStationLeft);
      intake->IntakeCoral();
      break;
    case ArmPosition::RIGHT_1:
      elevator->GoToPosition(setpoints::levelOneRight);
      intake->Stop();
      break;
    case ArmPosition::RIGHT_2:
      elevator->GoToPosition(setpoints::levelTwoRight);
      intake->Stop();
      break;
    case ArmPosition::RIGHT_3:
      elevator->GoToPosition(setpoints::levelThreeRight);
      intake->Stop();
      break;
    case ArmPosition::RIGHT_4:
      elevator->GoToPosition(setpoints::levelFourRight);
      intake->Stop();
      break;
    case ArmPosition::RIGHT_STATION:
      elevator->GoToPosition(setpoints::coralStationRight);
      intake->IntakeCoral();
      break;
    case ArmPosition::UNKNOWN:
      break;
  }
}

ArmPosition auto_utils::stringToPosition(const std::string str) {
  if (str == "S") {
    return ArmPosition::STOW;
  }
  if (str == "L1") {
    return ArmPosition::LEFT_1;
  }
  if (str == "L2") {
    return ArmPosition::LEFT_2;
  }
  if (str == "L3") {
    return ArmPosition::LEFT_3;
  }
  if (str == "L4") {
    return ArmPosition::LEFT_4;
  }
  if (str == "LS") {
    return ArmPosition::LEFT_STATION;
  }
  if (str == "R1") {
    return ArmPosition::RIGHT_1;
  }
  if (str == "R2") {
    return ArmPosition::RIGHT_2;
  }
  if (str == "R3") {
    return ArmPosition::RIGHT_3;
  }
  if (str == "R4") {
    return ArmPosition::RIGHT_4;
  }
  if (str == "RS") {
    return ArmPosition::RIGHT_STATION;
  }
  return ArmPosition::UNKNOWN;
}

std::string auto_utils::toString(ArmPosition position) {
  switch (position) {
    case (ArmPosition::STOW):
      return "S";
    case (ArmPosition::LEFT_1):
      return "L1";
    case (ArmPosition::LEFT_2):
      return "L2";
    case (ArmPosition::LEFT_3):
      return "L3";
    case (ArmPosition::LEFT_4):
      return "L4";
    case (ArmPosition::LEFT_STATION):
      return "LS";
    case (ArmPosition::RIGHT_1):
      return "R1";
    case (ArmPosition::RIGHT_2):
      return "R2";
    case (ArmPosition::RIGHT_3):
      return "R3";
    case (ArmPosition::RIGHT_4):
      return "R4";
    case (ArmPosition::RIGHT_STATION):
      return "RS";
    case (ArmPosition::UNKNOWN):
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

std::vector<ArmPosition> auto_utils::getPositions() {
  std::vector<ArmPosition> positions;
  const int numPositions = static_cast<int>(ArmPosition::UNKNOWN);
  positions.reserve(numPositions);
  for (int i = 0; i < numPositions; ++i) {
    positions.push_back(static_cast<ArmPosition>(i));
  }
  return positions;
}

std::vector<std::string> auto_utils::getPositionStrings() {
  auto positions = getPositions();
  std::vector<std::string> strPositions;
  strPositions.reserve(positions.size());
  std::transform(positions.begin(), positions.end(), std::back_inserter(strPositions), &toString);
  return strPositions;
}
