/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/climb_command.h"

ClimbCommand::ClimbCommand(ClimberSubsystem* climberSubsystem) : m_pClimberSubsystem{climberSubsystem} {
  AddRequirements(m_pClimberSubsystem);
}

// Called when the command is initially scheduled.
void ClimbCommand::Initialize() {
  // if (m_pClimberSubsystem->ClimberGetAngle() < 30_deg) {
  //   m_pClimberSubsystem->ClimberMoveToAngle(30_deg);
  // }
}

// Called repeatedly when this Command is scheduled to run
void ClimbCommand::Execute() {
  if (m_pClimberSubsystem->GetClimberManualOverride()) {
    Cancel();
  }
  m_pClimberSubsystem->WinchIn(0.75, false);
  if (m_pClimberSubsystem->GetPositionMotorCurrent() > 40_A) {
    m_pClimberSubsystem->PositionMotorStop();
  }
}

// Called once the command ends or is interrupted.
void ClimbCommand::End(bool interrupted) {
  m_pClimberSubsystem->ClimberStop();
}

// Returns true when the command should end.
bool ClimbCommand::IsFinished() {
  return m_pClimberSubsystem->ClimberGetAngle() >= 88_deg;
}
