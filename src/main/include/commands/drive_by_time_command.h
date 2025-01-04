/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <chrono>

#include "subsystems/swerve_drive_subsystem.h"

class DriveByTimeCommand : public frc2::CommandHelper<frc2::Command, DriveByTimeCommand> {
 public:
  /// @brief Constructs DriveByTimeCommand that drives by a vector with angle robotYaw and power percentSpeed for driveTime milliseconds
  /// @param swerveDrive swerve drive subsystem
  /// @param robotYaw Desired heading relative to field
  /// @param percentSpeed Desired percent speed of drivetrain
  /// @param driveTime Time to drive along vector
  DriveByTimeCommand(SwerveDriveSubsystem& swerveDrive,
                     units::degree_t robotYaw,
                     double percentSpeed,
                     units::millisecond_t driveTime);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDriveSubsystem& m_swerveDrive;
  units::degree_t m_robotYaw;
  double m_percentSpeed;
  units::millisecond_t m_driveTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
};
