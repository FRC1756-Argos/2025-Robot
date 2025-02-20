/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "controls/operator_controller.h"

OperatorController::OperatorController(int controllerID) : m_macropad{controllerID} {};

frc2::Trigger OperatorController::TriggerReefFlex() {
  return frc2::Trigger([&]() { m_macropad.GetRawButton(0); });
}
frc2::Trigger OperatorController::TriggerReefA() {}
frc2::Trigger OperatorController::TriggerReefB() {}
frc2::Trigger OperatorController::TriggerReefC() {}
frc2::Trigger OperatorController::TriggerReefD() {}
frc2::Trigger OperatorController::TriggerReefE() {}
frc2::Trigger OperatorController::TriggerReefF() {}
frc2::Trigger OperatorController::TriggerReefLeft() {}
frc2::Trigger OperatorController::TriggerReefRight() {}
frc2::Trigger OperatorController::TriggerStow() {}
frc2::Trigger OperatorController::TriggerCoral() {}
frc2::Trigger OperatorController::TriggerAlgae() {}
