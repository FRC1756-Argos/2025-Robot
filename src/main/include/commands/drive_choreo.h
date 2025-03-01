/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <choreo/trajectory/Trajectory.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/length.h>
#include <wpi/DataLog.h>

#include <chrono>
#include <optional>
#include <string>

#include "constants/position.h"
#include "subsystems/swerve_drive_subsystem.h"

class DriveChoreo : public frc2::CommandHelper<frc2::Command, DriveChoreo> {
 public:
  DriveChoreo(SwerveDriveSubsystem& drive,
              const std::string& trajectoryName,
              const bool initializeOdometry = false,
              std::optional<std::function<void(ArmPosition)>> armPositionCallback = std::nullopt);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  [[nodiscard]] static bool IsAtEndPoint(SwerveDriveSubsystem& drive,
                                         const std::string& trajectoryName,
                                         const units::inch_t translationalTolerance = 6_in,
                                         const units::degree_t rotationalTolerance = 1.0_deg);

 private:
  SwerveDriveSubsystem& m_Drive;
  const std::optional<choreo::Trajectory<choreo::SwerveSample>> m_trajectory;
  const bool m_initializeOdometry;
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;
  wpi::log::StructLogEntry<frc::Pose2d> m_desiredAutoPositionLogger;
  wpi::log::StructArrayLogEntry<frc::Pose2d> m_autoTrajectoryLogger;
  bool m_isRedAlliance;
  std::optional<std::function<void(ArmPosition)>> m_armPositionCallback;
  std::vector<choreo::EventMarker> m_events;
  size_t m_nextEventIndex;
};
