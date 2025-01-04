/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/auto_selector.h"

#include <networktables/NetworkTableInstance.h>

AutoSelector::AutoSelector() : m_log{"AUTO_SELECTOR"} {
  UpdateSelectorEntries();
}

AutoSelector::AutoSelector(std::initializer_list<AutonomousCommand*> commands, AutonomousCommand* defaultCommand)
    : m_commands{commands}, m_default{defaultCommand}, m_log{"AUTO_SELECTOR"} {
  UpdateSelectorEntries();
}

void AutoSelector::AddCommand(std::initializer_list<AutonomousCommand*> commands) {
  m_commands.insert(m_commands.end(), commands.begin(), commands.end());

  UpdateSelectorEntries();
}

void AutoSelector::AddCommand(AutonomousCommand* command) {
  AddCommand({command});
}

void AutoSelector::SetDefaultCommand(AutonomousCommand* defaultCommand) {
  m_default = defaultCommand;
}

frc2::Command* AutoSelector::GetSelectedCommand() const {
  std::string selectedRoutineName =
      nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetString("Auto Selector", "");

  const auto selectedRoutineIt =
      std::find_if(m_commands.begin(), m_commands.end(), [selectedRoutineName](AutonomousCommand* toCheck) {
        return toCheck != nullptr && selectedRoutineName == toCheck->GetName();
      });
  if (selectedRoutineIt != std::end(m_commands)) {
    const auto selectedRoutine = *selectedRoutineIt;
    return selectedRoutine->GetCommand();
  }
  // No match found
  if (m_default != nullptr) {
    return m_default->GetCommand();
  }

  // No valid default found, log error
  m_log.Log(argos_lib::LogLevel::ERR, "Default auto command is nullptr\n");

  // No valid default either... :(
  return nullptr;
}

void AutoSelector::UpdateSelectorEntries() const {
  std::vector<std::string> routines;
  std::transform(m_commands.begin(), m_commands.end(), std::back_inserter(routines), [](AutonomousCommand* command) {
    return command->GetName();
  });
  nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutStringArray("Auto List", routines);
}
