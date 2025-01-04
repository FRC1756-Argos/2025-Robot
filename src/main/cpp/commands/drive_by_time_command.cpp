/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_by_time_command.h"

#include "argos_lib/general/angle_utils.h"

DriveByTimeCommand::DriveByTimeCommand(SwerveDriveSubsystem& swerveDrive,
                                       units::degree_t robotYaw,
                                       double percentSpeed,
                                       units::millisecond_t driveTime)
    : m_swerveDrive{swerveDrive}, m_robotYaw{robotYaw}, m_driveTime{driveTime} {
  m_percentSpeed = std::clamp<double>(percentSpeed, 0, 1.0);
  argos_lib::angle::ConstrainAngle(robotYaw, 0_deg, 360_deg);

  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&m_swerveDrive);
}

// Called when the command is initially scheduled.
void DriveByTimeCommand::Initialize() {
  m_startTime = std::chrono::high_resolution_clock::now();
}

// Called repeatedly when this Command is scheduled to run
void DriveByTimeCommand::Execute() {
  m_swerveDrive.SwerveDrive(m_robotYaw, m_percentSpeed);
}

// Called once the command ends or is interrupted.
void DriveByTimeCommand::End(bool interrupted) {
  m_swerveDrive.StopDrive();
}

// Returns true when the command should end.
bool DriveByTimeCommand::IsFinished() {
  auto currentTime = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - m_startTime).count() >=
         m_driveTime.to<double>();
}
