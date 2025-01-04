/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_choreo.h"

/// @todo Use 2025 ChoreoLib
// #include <choreo/Choreo.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <networktables/StructTopic.h>
#include <units/math.h>

#include "constants/field_points.h"

DriveChoreo::DriveChoreo(SwerveDriveSubsystem& drive, const std::string& trajectoryName, const bool initializeOdometry)
    : m_Drive{drive}  // , m_trajectory{choreolib::Choreo::GetTrajectory(trajectoryName)}
    // , m_ChoreoCommand{m_trajectory,
    //                   [&drive]() { return drive.GetRawOdometry(); },
    //                   drive.GetChoreoControllerFunction(),
    //                   [&drive](frc::ChassisSpeeds speeds) { return drive.SwerveDrive(speeds); },
    //                   []() {
    //                     const auto alliance = frc::DriverStation::GetAlliance();
    //                     return alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
    //                   },
    //                   {&m_Drive}}
    , m_initializeOdometry{initializeOdometry}
    , m_desiredAutoPositionLogger{frc::DataLogManager::GetLog(), "desiredAutoPosition"}
    , m_autoTrajectoryLogger{frc::DataLogManager::GetLog(), "autoTrajectory"} {
  frc::DataLogManager::GetLog().AddStructSchema<frc::Pose2d>();
}

// Called when the command is initially scheduled.
void DriveChoreo::Initialize() {
  // Initial odometry changes base on alliance because choreo always uses odometry relative to blue alliance origin
  // const auto alliance = frc::DriverStation::GetAlliance();
  // if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
  //   if (m_initializeOdometry) {
  //     m_Drive.InitializeOdometry(m_trajectory.GetFlippedInitialPose());
  //   }
  //   m_orientedTrajectory = m_trajectory.Flipped();
  // } else {
  //   if (m_initializeOdometry) {
  //     m_Drive.InitializeOdometry(m_trajectory.GetInitialPose());
  //   }
  //   m_orientedTrajectory = m_trajectory;
  // }
  // Driver still wants orientation relative to alliance station
  // if (m_initializeOdometry) {
  //   if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
  //     m_Drive.FieldHome(-m_trajectory.GetInitialPose().Rotation().Degrees(), false);
  //   } else {
  //     m_Drive.FieldHome(m_trajectory.GetInitialPose().Rotation().Degrees(), false);
  //   }
  // }
  // std::vector<frc::Pose2d> trajectory;
  // trajectory.reserve(m_orientedTrajectory.GetSamples().size());

  // if (!m_orientedTrajectory.GetSamples().empty()) {
  //   for (const auto& sample : m_orientedTrajectory.GetSamples()) {
  //     trajectory.emplace_back(sample.GetPose());
  //   }
  // }

  // if (!trajectory.empty()) {
  //   m_autoTrajectoryLogger.Append(trajectory);
  // }

  // m_startTime = std::chrono::steady_clock::now();
  // m_ChoreoCommand.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void DriveChoreo::Execute() {
  // m_ChoreoCommand.Execute();
  // if (!m_orientedTrajectory.GetSamples().empty()) {
  //   m_desiredAutoPositionLogger.Append(
  //       m_orientedTrajectory.Sample(std::chrono::steady_clock::now() - m_startTime).GetPose());
  // }
}

// Called once the command ends or is interrupted.
void DriveChoreo::End(bool interrupted) {
  m_autoTrajectoryLogger.Append(std::vector<frc::Pose2d>{});
  // m_ChoreoCommand.End(interrupted);
}

// Returns true when the command should end.
bool DriveChoreo::IsFinished() {
  // return m_ChoreoCommand.IsFinished();
  return true;
}

bool DriveChoreo::IsAtEndPoint(SwerveDriveSubsystem& drive,
                               const std::string& trajectoryName,
                               const units::inch_t translationalTolerance,
                               const units::degree_t rotationalTolerance) {
  // const auto position = drive.GetRawOdometry();
  // const auto alliance = frc::DriverStation::GetAlliance();
  // const auto needFlipped = alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
  // const auto trajectory = choreolib::Choreo::GetTrajectory(trajectoryName);
  // const auto desiredPosition = needFlipped ? trajectory.GetFlippedFinalPose() : trajectory.GetFinalPose();
  // return units::math::abs((position.Rotation() - desiredPosition.Rotation()).Degrees()) <= rotationalTolerance &&
  //        units::math::abs((position.Translation() - desiredPosition.Translation()).Norm()) <= translationalTolerance;
  return true;
}

units::inch_t DriveChoreo::EndpointShotDistance(const std::string& trajectoryName) {
  // return units::math::abs((choreolib::Choreo::GetTrajectory(trajectoryName).GetFinalPose().Translation() -
  //                          field_points::blue_alliance::april_tags::speakerCenter.pose.ToTranslation2d())
  //                             .Norm()) +
  //        10_in;  // 10 inches converts camera position to center of robot
  return 0_in;
}
