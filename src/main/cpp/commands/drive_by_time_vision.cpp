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
    : m_swerveDrive{swerveDrive}
    , m_visionSubsystem{visionSubsystem}
    , m_driveTime{driveTime}
    , m_autoAimDebouncer{150_ms} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({&m_swerveDrive, &m_visionSubsystem});
}

// Called when the command is initially scheduled.
void DriveByTimeVisionCommand::Initialize() {
  m_startTime = std::chrono::high_resolution_clock::now();
  m_autoAimDebouncer.Reset(false);
}

// Called repeatedly when this Command is scheduled to run
void DriveByTimeVisionCommand::Execute() {
  auto speeds = m_visionSubsystem.getVisionAlignmentSpeeds(0.7);

  if (frc::DriverStation::IsAutonomous()) {
    if (speeds) {
      m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::robotCentricControl);
      m_swerveDrive.SwerveDrive(
          speeds.value().forwardSpeed,
          speeds.value().leftSpeed,
          speeds.value().ccwSpeed);  // X axis is positive right (CW), but swerve coordinates are positive left (CCW)
    } else {
      m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::fieldCentricControl);
    }
  }
}

// Called once the command ends or is interrupted.
void DriveByTimeVisionCommand::End(bool interrupted) {
  m_swerveDrive.StopDrive();
  m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::fieldCentricControl);
  m_visionSubsystem.SetLeftAlign(false);
  m_visionSubsystem.SetRightAlign(false);
  m_visionSubsystem.SetAlgaeAlign(false);
}

// Returns true when the command should end.
bool DriveByTimeVisionCommand::IsFinished() {
  auto currentTime = std::chrono::high_resolution_clock::now();
  auto error = m_visionSubsystem.GetRobotSpaceReefAlignmentError();  // End early if aligned
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - m_startTime).count();
  auto debouncedStatus = m_autoAimDebouncer.GetDebouncedStatus();
  if (error) {
    debouncedStatus =
        m_autoAimDebouncer(units::math::abs(error.value().Norm()) <= measure_up::reef::reefValidAlignmentDistance);
  }
  return (duration >= m_driveTime.to<double>()) || debouncedStatus;
}
