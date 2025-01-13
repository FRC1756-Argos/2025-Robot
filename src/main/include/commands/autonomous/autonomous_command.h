/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>

#include <string>

/**
 * @brief A command that can be selected from the dashboard
 */
class AutonomousCommand {
 public:
  AutonomousCommand() = default;

  /**
   * @brief Get the name of the command to display on the dashboard
   *
   * @return std::string Name to display
   */
  virtual std::string GetName() const = 0;

  /**
   * @brief Get the command to run when selected from dashboard
   *
   * @return frc2::Command* Command to run
   */
  virtual frc2::Command* GetCommand() = 0;
};
