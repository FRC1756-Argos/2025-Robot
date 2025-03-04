/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <chrono>

#include "Constants.h"
#include "constants/measure_up.h"
#include "subsystems/swerve_drive_subsystem.h"
#include "subsystems/vision_subsystem.h"

class DriveByTimeVisionCommand : public frc2::CommandHelper<frc2::Command, DriveByTimeVisionCommand> {
 public:
  /// @brief Constructs DriveByTimeCommand that drives by a vector with angle robotYaw and power percentSpeed for driveTime milliseconds
  /// @param swerveDrive swerve drive subsystem
  /// @param robotYaw Desired heading relative to field
  /// @param percentSpeed Desired percent speed of drivetrain
  /// @param driveTime Time to drive along vector
  DriveByTimeVisionCommand(SwerveDriveSubsystem& swerveDrive,
                           VisionSubsystem& visionSubsystem,
                           bool leftAlignment,
                           units::millisecond_t driveTime);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDriveSubsystem& m_swerveDrive;
  VisionSubsystem& m_visionSubsystem;
  bool LeftAlignment;
  units::millisecond_t m_driveTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
};
