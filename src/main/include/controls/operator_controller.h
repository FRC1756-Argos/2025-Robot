/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/GenericHID.h>
#include <frc2/command/button/Trigger.h>

class OperatorController {
 public:
  explicit OperatorController(int controllerID);
  OperatorController() = delete;

  enum class GamePieceMode { Algae, Coral };
  enum class ReefLevel { L1, L2, L3, L4 };
  enum class ArmDirection { Left, Right };

  [[nodiscard]] frc2::Trigger TriggerL1();
  [[nodiscard]] frc2::Trigger TriggerL2();
  [[nodiscard]] frc2::Trigger TriggerL3();
  [[nodiscard]] frc2::Trigger TriggerL4();
  [[nodiscard]] frc2::Trigger TriggerReefLeft();
  [[nodiscard]] frc2::Trigger TriggerReefRight();
  [[nodiscard]] frc2::Trigger TriggerStow();
  [[nodiscard]] frc2::Trigger TriggerCoral();
  [[nodiscard]] frc2::Trigger TriggerAlgae();

  [[nodiscard]] GamePieceMode GetGamePieceMode();
  [[nodiscard]] ArmDirection GetArmDirection();
  [[nodiscard]] ReefLevel GetReefLevel();

 private:
  frc::GenericHID m_macropad;
  ArmDirection m_activeDirection;
  ReefLevel m_activeLevel;
};
