/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/swerve_drive_subsystem.h"

class InitializeOdometryCommand : public frc2::CommandHelper<frc2::Command, InitializeOdometryCommand> {
 public:
  InitializeOdometryCommand(SwerveDriveSubsystem* pDrive, frc::Pose2d initialPose);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDriveSubsystem* m_pDrive;
  frc::Pose2d m_initialPose;
};
