/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "controls/operator_controller.h"

#include <frc2/command/InstantCommand.h>

OperatorController::OperatorController(int controllerID)
    : m_macropad{controllerID}
    , m_activeDirection{OperatorController::ArmDirection::Left}
    , m_activeLevel{OperatorController::ReefLevel::L4} {
  TriggerReefLeft().OnTrue(frc2::InstantCommand([&]() { m_activeDirection = ArmDirection::Left; }, {}).ToPtr());
  TriggerReefRight().OnTrue(frc2::InstantCommand([&]() { m_activeDirection = ArmDirection::Right; }, {}).ToPtr());
  TriggerL1().OnTrue(frc2::InstantCommand([&]() { m_activeLevel = ReefLevel::L1; }, {}).ToPtr());
  TriggerL2().OnTrue(frc2::InstantCommand([&]() { m_activeLevel = ReefLevel::L2; }, {}).ToPtr());
  TriggerL3().OnTrue(frc2::InstantCommand([&]() { m_activeLevel = ReefLevel::L3; }, {}).ToPtr());
  TriggerL4().OnTrue(frc2::InstantCommand([&]() { m_activeLevel = ReefLevel::L4; }, {}).ToPtr());
};

frc2::Trigger OperatorController::TriggerL1() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(12); });
}
frc2::Trigger OperatorController::TriggerL2() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(9); });
}
frc2::Trigger OperatorController::TriggerL3() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(6); });
}
frc2::Trigger OperatorController::TriggerL4() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(3); });
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
  return frc2::Trigger([&]() { return !m_macropad.GetRawButton(1); });
}
frc2::Trigger OperatorController::TriggerAlgae() {
  return frc2::Trigger([&]() { return m_macropad.GetRawButton(10); });
}

OperatorController::GamePieceMode OperatorController::GetGamePieceMode() {
  return m_macropad.GetRawButton(10) ? GamePieceMode::Algae : GamePieceMode::Coral;
}

OperatorController::ArmDirection OperatorController::GetArmDirection() {
  return m_activeDirection;
}

OperatorController::ReefLevel OperatorController::GetReefLevel() {
  return m_activeLevel;
}
