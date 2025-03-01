/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/climb_command.h"

ClimbCommand::ClimbCommand(ClimberSubsystem* climberSubsystem) : m_pClimberSubsystem{climberSubsystem} {
  AddRequirements(m_pClimberSubsystem);
}

// Called when the command is initially scheduled.
void ClimbCommand::Initialize() {
  if (m_pClimberSubsystem->ClimberGetAngle() < 25_deg) {
    m_pClimberSubsystem->ClimberMoveToAngle(25_deg);
  }
}

// Called repeatedly when this Command is scheduled to run
void ClimbCommand::Execute() {
  if (m_pClimberSubsystem->GetClimberManualOverride()) {
    Cancel();
  }
  if (m_pClimberSubsystem->ClimberGetAngle() >= 25_deg) {
    m_pClimberSubsystem->WinchIn();
  }
  if (m_pClimberSubsystem->ClimberGetAngle() >= 30_deg) {
    m_pClimberSubsystem->SetPositionMotorBreakModeToBreak(false);
  }
}

// Called once the command ends or is interrupted.
void ClimbCommand::End(bool interrupted) {
  m_pClimberSubsystem->ClimberStop();
}

// Returns true when the command should end.
bool ClimbCommand::IsFinished() {
  return m_pClimberSubsystem->ClimberGetAngle() >= 95_deg;
}
