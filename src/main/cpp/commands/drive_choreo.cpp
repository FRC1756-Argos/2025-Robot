/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_choreo.h"

#include <choreo/Choreo.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <networktables/StructTopic.h>
#include <units/math.h>

#include "constants/field_points.h"

DriveChoreo::DriveChoreo(SwerveDriveSubsystem& drive, const std::string& trajectoryName, const bool initializeOdometry)
    : m_Drive{drive}
    , m_trajectory{choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(trajectoryName)}
    , m_initializeOdometry{initializeOdometry}
    , m_desiredAutoPositionLogger{frc::DataLogManager::GetLog(), "desiredAutoPosition"}
    , m_autoTrajectoryLogger{frc::DataLogManager::GetLog(), "autoTrajectory"} {
  frc::DataLogManager::GetLog().AddStructSchema<frc::Pose2d>();
}

// Called when the command is initially scheduled.
void DriveChoreo::Initialize() {
  // Initial odometry changes base on alliance because choreo always uses odometry relative to blue alliance origin
  const auto alliance = frc::DriverStation::GetAlliance();
  if (m_trajectory) {
    if (m_initializeOdometry) {
      m_Drive.InitializeOdometry(m_trajectory.value()
                                     .GetInitialPose(alliance && alliance.value() == frc::DriverStation::Alliance::kRed)
                                     .value());
    }
    if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
      m_orientedTrajectory = m_trajectory.value().Flipped();
    } else {
      m_orientedTrajectory = m_trajectory.value();
    }
  }
  // Driver still wants orientation relative to alliance station
  if (m_initializeOdometry && m_trajectory) {
    if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
      m_Drive.FieldHome(-m_trajectory.value().GetInitialPose().value().Rotation().Degrees(), false);
    } else {
      m_Drive.FieldHome(m_trajectory.value().GetInitialPose().value().Rotation().Degrees(), false);
    }
  }
  std::vector<frc::Pose2d> trajectory;
  trajectory.reserve(m_orientedTrajectory.samples.size());

  if (!m_orientedTrajectory.samples.empty()) {
    for (const auto& sample : m_orientedTrajectory.samples) {
      trajectory.emplace_back(sample.GetPose());
    }
  }

  if (!trajectory.empty()) {
    m_autoTrajectoryLogger.Append(trajectory);
  }

  m_startTime = std::chrono::steady_clock::now();
}

// Called repeatedly when this Command is scheduled to run
void DriveChoreo::Execute() {
  m_Drive.SwerveDrive(m_orientedTrajectory.SampleAt(std::chrono::steady_clock::now() - m_startTime).value());
  if (!m_orientedTrajectory.samples.empty()) {
    m_desiredAutoPositionLogger.Append(
        m_orientedTrajectory.SampleAt(std::chrono::steady_clock::now() - m_startTime).value().GetPose());
  }
}

// Called once the command ends or is interrupted.
void DriveChoreo::End(bool interrupted) {
  m_autoTrajectoryLogger.Append(std::vector<frc::Pose2d>{});
  m_Drive.StopDrive();
}

// Returns true when the command should end.
bool DriveChoreo::IsFinished() {
  return units::millisecond_t(std::chrono::steady_clock::now() - m_startTime) >= m_orientedTrajectory.GetTotalTime();
}

bool DriveChoreo::IsAtEndPoint(SwerveDriveSubsystem& drive,
                               const std::string& trajectoryName,
                               const units::inch_t translationalTolerance,
                               const units::degree_t rotationalTolerance) {
  const auto position = drive.GetRawOdometry();
  const auto alliance = frc::DriverStation::GetAlliance();
  const auto needFlipped = alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
  const auto trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(trajectoryName);
  const auto desiredPosition = trajectory.value().GetFinalPose(needFlipped).value();
  return units::math::abs((position.Rotation() - desiredPosition.Rotation()).Degrees()) <= rotationalTolerance &&
         units::math::abs((position.Translation() - desiredPosition.Translation()).Norm()) <= translationalTolerance;
}
