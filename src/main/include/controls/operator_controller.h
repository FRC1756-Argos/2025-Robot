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

  [[nodiscard]] frc2::Trigger TriggerReefFlex();
  [[nodiscard]] frc2::Trigger TriggerReefA();
  [[nodiscard]] frc2::Trigger TriggerReefB();
  [[nodiscard]] frc2::Trigger TriggerReefC();
  [[nodiscard]] frc2::Trigger TriggerReefD();
  [[nodiscard]] frc2::Trigger TriggerReefE();
  [[nodiscard]] frc2::Trigger TriggerReefF();
  [[nodiscard]] frc2::Trigger TriggerReefLeft();
  [[nodiscard]] frc2::Trigger TriggerReefRight();
  [[nodiscard]] frc2::Trigger TriggerStow();
  [[nodiscard]] frc2::Trigger TriggerCoral();
  [[nodiscard]] frc2::Trigger TriggerAlgae();

 private:
  frc::GenericHID m_macropad;
};
