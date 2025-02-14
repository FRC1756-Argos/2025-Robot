/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_choreo_test.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <units/length.h>

#include "commands/drive_choreo.h"

AutonomousChoreoTest::AutonomousChoreoTest(ElevatorSubsystem& elevator,
                                           IntakeSubsystem& intake,
                                           SwerveDriveSubsystem& swerve,
                                           VisionSubsystem& vision)
    : m_Elevator{elevator}
    , m_Intake{intake}
    , m_Swerve{swerve}
    , m_Vision{vision}
    , m_allCommands{frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "TestPath.1", true}}} {}

// Called when the command is initially scheduled.
void AutonomousChoreoTest::Initialize() {
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousChoreoTest::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousChoreoTest::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousChoreoTest::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousChoreoTest::GetName() const {
  return "99. Choreo Test";
}

frc2::Command* AutonomousChoreoTest::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
