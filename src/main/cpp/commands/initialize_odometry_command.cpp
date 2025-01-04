/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/initialize_odometry_command.h"

InitializeOdometryCommand::InitializeOdometryCommand(SwerveDriveSubsystem* pDrive, frc::Pose2d initialPose)
    : m_pDrive{pDrive}, m_initialPose{initialPose} {
  AddRequirements(pDrive);
}

// Called when the command is initially scheduled.
void InitializeOdometryCommand::Initialize() {
  m_pDrive->InitializeOdometry(m_initialPose);
}

// Called repeatedly when this Command is scheduled to run
void InitializeOdometryCommand::Execute() {}

// Called once the command ends or is interrupted.
void InitializeOdometryCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool InitializeOdometryCommand::IsFinished() {
  return true;
}
