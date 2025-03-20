/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_by_time_vision.h"

#include <frc/DriverStation.h>
#include <units/length.h>

#include <cmath>

#include "argos_lib/general/angle_utils.h"
#include "constants/measure_up.h"

DriveByTimeVisionCommand::DriveByTimeVisionCommand(SwerveDriveSubsystem& swerveDrive,
                                                   VisionSubsystem& visionSubsystem,
                                                   bool leftAlignment,
                                                   units::millisecond_t driveTime)
    : m_swerveDrive{swerveDrive}, m_visionSubsystem{visionSubsystem}, m_driveTime{driveTime} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({&m_swerveDrive, &m_visionSubsystem});
}

// Called when the command is initially scheduled.
void DriveByTimeVisionCommand::Initialize() {
  m_startTime = std::chrono::high_resolution_clock::now();
}

// Called repeatedly when this Command is scheduled to run
void DriveByTimeVisionCommand::Execute() {
  double forwardSpeed = 0.0;
  double leftSpeed = 0.0;
  double rotateSpeed = 0.0;

  if (m_visionSubsystem.LeftAlignmentRequested() || m_visionSubsystem.RightAlignmentRequested()) {
    auto robotToTagCorrections = m_visionSubsystem.GetRobotSpaceReefAlignmentError();
    auto robotRotationCorrection = m_visionSubsystem.GetOrientationCorrection();
    if (robotToTagCorrections && robotRotationCorrection) {
      m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::robotCentricControl);
      units::meter_t forwardCorrection = robotToTagCorrections.value().X();
      units::degree_t rotationCorrection = robotRotationCorrection.value();
      units::meter_t lateralCorrection = robotToTagCorrections.value().Y();

      rotateSpeed = -speeds::drive::rotationalProportionality * rotationCorrection.value();

      // once we are almost oriented parallel to reef start zeroing down on the desired speeds
      if (units::math::abs(rotationCorrection) < 10.0_deg) {
        if (units::math::abs(lateralCorrection) > measure_up::reef::reefErrorFloorForward) {
          forwardSpeed = speeds::drive::translationalProportionality * (lateralCorrection.value());
          if (std::abs(forwardSpeed) < measure_up::reef::visionMinSpeed) {
            forwardSpeed = (forwardSpeed < 0.0 ? -1.0 : 1.0) * measure_up::reef::visionMinSpeed;
          } else if (std::abs(forwardSpeed) > measure_up::reef::visionMaxLongitudinalSpeed) {
            forwardSpeed = (forwardSpeed < 0.0 ? -1.0 : 1.0) * measure_up::reef::visionMaxLongitudinalSpeed;
          }
        } else {
          forwardSpeed = 0;
        }
        if (units::math::abs(forwardCorrection) > measure_up::reef::reefErrorFloorLat) {
          leftSpeed = -speeds::drive::translationalProportionality * (forwardCorrection.value());
          if (std::abs(leftSpeed) < measure_up::reef::visionMinSpeed) {
            leftSpeed = (leftSpeed < 0.0 ? -1.0 : 1.0) * measure_up::reef::visionMinSpeed;
          }
        } else {
          leftSpeed = 0;
        }
      }
    } else {
      m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::fieldCentricControl);
    }
  } else {
    m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::fieldCentricControl);
  }

  if (frc::DriverStation::IsAutonomous()) {
    m_swerveDrive.SwerveDrive(
        forwardSpeed,
        leftSpeed,
        rotateSpeed);  // X axis is positive right (CW), but swerve coordinates are positive left (CCW)
  }
}

// Called once the command ends or is interrupted.
void DriveByTimeVisionCommand::End(bool interrupted) {
  m_swerveDrive.StopDrive();
  m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::fieldCentricControl);
  m_visionSubsystem.SetLeftAlign(false);
  m_visionSubsystem.SetRightAlign(false);
}

// Returns true when the command should end.
bool DriveByTimeVisionCommand::IsFinished() {
  auto currentTime = std::chrono::high_resolution_clock::now();
  auto error = m_visionSubsystem.GetRobotSpaceReefAlignmentError();  // End early if aligned
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - m_startTime).count();
  return (duration >= m_driveTime.to<double>()) ||
         (error && duration > 100 &&
          units::math::abs(error.value().Norm()) <= measure_up::reef::reefValidAlignmentDistance);
}
