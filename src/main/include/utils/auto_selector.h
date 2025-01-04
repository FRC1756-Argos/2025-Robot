/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/general/log.h>
#include <commands/autonomous/autonomous_command.h>
#include <frc2/command/Command.h>

#include <initializer_list>
#include <vector>

/**
 * @brief Allow user to select from auto routines using the default dashboard autonomous selector.
 */
class AutoSelector {
 public:
  /**
   * @brief Initialize with no available auto routines.  More may be added later.
   */
  AutoSelector();

  /**
   * @brief Initialize with specified commands.  More may be added later.
   *
   * @param commands List of commands to allow user to select from
   * @param defaultCommand Command to run if selection is invalid.  nullptr indicates run no command.
   */
  AutoSelector(std::initializer_list<AutonomousCommand*> commands, AutonomousCommand* defaultCommand = nullptr);

  /**
   * @brief Add a set of commands to the selector.
   *
   * @param commands Commands to add
   */
  void AddCommand(std::initializer_list<AutonomousCommand*> commands);

  /**
   * @brief Add a single command to the selector.
   *
   * @param command Command to add
   */
  void AddCommand(AutonomousCommand* command);

  /**
   * @brief Set the default command to run if user selection is invalid
   *
   * @param defaultCommand Default command.  nullptr indicates run no command
   */
  void SetDefaultCommand(AutonomousCommand* defaultCommand);

  /**
   * @brief Get the command selected by the user on the dashboard.  Will return default command if
   *        no match is found.
   *
   * @return frc2::Command* Selected command
   */
  frc2::Command* GetSelectedCommand() const;

 private:
  std::vector<AutonomousCommand*> m_commands;  ///< All autonomous commands that can be selected
  AutonomousCommand* m_default;                ///< Command to run if dashboard selection is invalid
  argos_lib::ArgosLogger m_log;

  /**
   * @brief Update dashboard with latest command list
   */
  void UpdateSelectorEntries() const;
};
