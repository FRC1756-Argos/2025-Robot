/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/commands/swap_controllers_command.h"

using namespace argos_lib;

SwapControllersCommand::SwapControllersCommand(SwappableControllersSubsystem* controllers)
    : m_pControllerSubsystem(controllers) {}

void SwapControllersCommand::Initialize() {
  m_pControllerSubsystem->VibrateAll(argos_lib::VibrationConstant(1));
}

void SwapControllersCommand::End(bool) {
  m_pControllerSubsystem->Swap();
  m_pControllerSubsystem->VibrateAll(argos_lib::VibrationOff());
}
