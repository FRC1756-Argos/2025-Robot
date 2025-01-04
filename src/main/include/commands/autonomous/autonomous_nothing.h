/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <string>

#include "commands/autonomous/autonomous_command.h"
#include "commands/initialize_odometry_command.h"
#include "subsystems/swerve_drive_subsystem.h"

class AutonomousNothing
    : public frc2::CommandHelper<frc2::Command, AutonomousNothing>
    , public AutonomousCommand {
 public:
  explicit AutonomousNothing(SwerveDriveSubsystem& swerve);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  /**
   * @copydoc AutonomousCommand::GetName()
   */
  std::string GetName() const final;
  /**
   * @copydoc AutonomousCommand::GetCommand()
   */
  frc2::Command* GetCommand() final;

 private:
  SwerveDriveSubsystem& m_swerve;
  InitializeOdometryCommand m_initializeOdometryRed;
  InitializeOdometryCommand m_initializeOdometryBlue;
};
