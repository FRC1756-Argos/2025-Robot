// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/climb_command.h"

ClimbCommand::ClimbCommand(ClimberSubsystem* climberSubsystem)
  : m_pClimberSubsystem{climberSubsystem} {
  AddRequirements(m_pClimberSubsystem);
}

// Called when the command is initially scheduled.
void ClimbCommand::Initialize() {
  m_pClimberSubsystem->WinchIn();
}

// Called repeatedly when this Command is scheduled to run
void ClimbCommand::Execute() {
  if (m_pClimberSubsystem->GetClimberManualOverride()) {
    Cancel();
  }
}

// Called once the command ends or is interrupted.
void ClimbCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimbCommand::IsFinished() {
  return m_pClimberSubsystem->ClimberGetAngle() >= 100_deg;
}
