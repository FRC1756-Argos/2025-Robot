/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/subsystems/swappable_controllers_subsystem.h"

using namespace argos_lib;

SwappableControllersSubsystem::SwappableControllersSubsystem(int driverControllerPort, int operatorControllerPort)
    : m_driverController(driverControllerPort), m_operatorController(operatorControllerPort), m_swapped(false) {}

void SwappableControllersSubsystem::Swap() {
  m_driverController.SwapSettings(m_operatorController);
  m_swapped = !m_swapped;
}

XboxController& SwappableControllersSubsystem::DriverController() {
  return m_swapped ? m_operatorController : m_driverController;
}
XboxController& SwappableControllersSubsystem::OperatorController() {
  return m_swapped ? m_driverController : m_operatorController;
}

/**
 * Will be called periodically whenever the CommandScheduler runs.
 */
void SwappableControllersSubsystem::Periodic() {
  UpdateVibration();
}

void SwappableControllersSubsystem::VibrateAll(VibrationModel newModel) {
  m_driverController.SetVibration(newModel);
  m_operatorController.SetVibration(newModel);
}

void SwappableControllersSubsystem::UpdateVibration() {
  m_driverController.UpdateVibration();
  m_operatorController.UpdateVibration();
}
