/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "controls/operator_controller.h"

OperatorController::OperatorController(int controllerID) : m_macropad{controllerID} {};

frc2::Trigger OperatorController::TriggerReefFlex() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(6); });
}
frc2::Trigger OperatorController::TriggerReefA() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(4); });
}
frc2::Trigger OperatorController::TriggerReefB() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(3); });
}
frc2::Trigger OperatorController::TriggerReefC() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(0); });
}
frc2::Trigger OperatorController::TriggerReefD() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(1); });
}
frc2::Trigger OperatorController::TriggerReefE() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(2); });
}
frc2::Trigger OperatorController::TriggerReefF() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(5); });
}
frc2::Trigger OperatorController::TriggerReefLeft() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(7); });
}
frc2::Trigger OperatorController::TriggerReefRight() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(8); });
}
frc2::Trigger OperatorController::TriggerStow() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(11); });
}
frc2::Trigger OperatorController::TriggerCoral() {
  return frc2::Trigger([&]() { return !m_macropad.GetRawButton(9); });
}
frc2::Trigger OperatorController::TriggerAlgae() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(9); });
}
