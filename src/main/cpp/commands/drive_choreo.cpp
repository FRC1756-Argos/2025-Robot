/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_choreo.h"

#include <choreo/Choreo.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <networktables/StructTopic.h>
#include <units/math.h>

#include "commands/autonomous/auto_utils.h"
#include "constants/field_points.h"

DriveChoreo::DriveChoreo(SwerveDriveSubsystem& drive,
                         const std::string& trajectoryName,
                         const bool initializeOdometry,
                         std::optional<std::function<void(ArmPosition)>> armPositionCallback,
                         const int split)
    : m_Drive{drive}
    , m_trajectory{choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(trajectoryName)}
    , m_initializeOdometry{initializeOdometry}
    , m_desiredAutoPositionLogger{frc::DataLogManager::GetLog(), "desiredAutoPosition"}
    , m_autoTrajectoryLogger{frc::DataLogManager::GetLog(), "autoTrajectory"}
    , m_isRedAlliance{false}
    , m_armPositionCallback{armPositionCallback}
    , m_events{}
    , m_nextEventIndex{0} {
  frc::DataLogManager::GetLog().AddStructSchema<frc::Pose2d>();
  if (m_armPositionCallback && m_trajectory) {
    if (split > 0 && static_cast<size_t>(split) < m_trajectory.value().splits.size() + 1) {
      m_trajectory = m_trajectory.value().GetSplit(split);
    }
    for (const auto& position : auto_utils::getPositionStrings()) {
      auto newEvents = m_trajectory.value().GetEvents(position);
      m_events.insert(std::end(m_events), newEvents.begin(), newEvents.end());
    }
    std::sort(m_events.begin(), m_events.end(), [](const choreo::EventMarker lhs, const choreo::EventMarker rhs) {
      return lhs.timestamp < rhs.timestamp;
    });
  }
}

// Called when the command is initially scheduled.
void DriveChoreo::Initialize() {
  // Initial odometry changes base on alliance because choreo always uses odometry relative to blue alliance origin
  const auto alliance = frc::DriverStation::GetAlliance();
  m_isRedAlliance = alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
  if (m_trajectory) {
    if (m_initializeOdometry) {
      m_Drive.InitializeOdometry(m_trajectory.value().GetInitialPose(m_isRedAlliance).value());
    }
  }
  // Driver still wants orientation relative to alliance station
  if (m_initializeOdometry && m_trajectory) {
    if (!m_isRedAlliance) {
      m_Drive.FieldHome(-m_trajectory.value().GetInitialPose(m_isRedAlliance).value().Rotation().Degrees(), false);
    } else {
      /// @todo This rotation shouldn't be necessary.  Probably has something to do with the negative sign
      m_Drive.FieldHome(
          -m_trajectory.value().GetInitialPose(m_isRedAlliance).value().Rotation().RotateBy(180_deg).Degrees(), false);
    }
  }
  std::vector<frc::Pose2d> trajectory;
  trajectory.reserve(m_trajectory.value().samples.size());
  if (m_isRedAlliance) {
    trajectory = m_trajectory.value().Flipped().GetPoses();
  } else {
    trajectory = m_trajectory.value().GetPoses();
  }

  if (!trajectory.empty()) {
    m_autoTrajectoryLogger.Append(trajectory);
    // m_Drive.SetTrajectoryDisplay(trajectory);
  }

  m_startTime = std::chrono::steady_clock::now();
  m_nextEventIndex = 0;
}

// Called repeatedly when this Command is scheduled to run
void DriveChoreo::Execute() {
  auto currentTime = std::chrono::steady_clock::now() - m_startTime;
  auto currentTarget = m_trajectory.value().SampleAt(currentTime, m_isRedAlliance).value();
  m_Drive.SwerveDrive(currentTarget);
  if (m_trajectory && !m_trajectory.value().samples.empty()) {
    m_desiredAutoPositionLogger.Append(currentTarget.GetPose());
  }
  if (!m_events.empty()) {
    while (m_nextEventIndex < m_events.size() &&
           units::time::millisecond_t(currentTime) >= m_events.at(m_nextEventIndex).timestamp) {
      m_armPositionCallback.value()(auto_utils::stringToPosition(m_events.at(m_nextEventIndex).event));
      ++m_nextEventIndex;
    }
  }
}

// Called once the command ends or is interrupted.
void DriveChoreo::End(bool interrupted) {
  m_autoTrajectoryLogger.Append(std::vector<frc::Pose2d>{});
  const auto endVelocity = m_trajectory.value().GetFinalSample().value().GetChassisSpeeds();
  if (interrupted || (units::math::abs(endVelocity.vx) < 0.01_fps && units::math::abs(endVelocity.vy) < 0.01_fps)) {
    m_Drive.StopDrive();
  }
}

// Returns true when the command should end.
bool DriveChoreo::IsFinished() {
  return units::millisecond_t(std::chrono::steady_clock::now() - m_startTime) >= m_trajectory.value().GetTotalTime();
}

bool DriveChoreo::IsAtEndPoint(SwerveDriveSubsystem& drive,
                               const std::string& trajectoryName,
                               const units::inch_t translationalTolerance,
                               const units::degree_t rotationalTolerance) {
  const auto position = drive.GetRawOdometry();
  const auto alliance = frc::DriverStation::GetAlliance();
  const auto isRedAlliance = alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
  const auto trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(trajectoryName);
  const auto desiredPosition = trajectory.value().GetFinalPose(isRedAlliance).value();
  return units::math::abs((position.Rotation() - desiredPosition.Rotation()).Degrees()) <= rotationalTolerance &&
         units::math::abs((position.Translation() - desiredPosition.Translation()).Norm()) <= translationalTolerance;
}
