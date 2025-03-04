/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

/// @copyriFEt CopyriFEt (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.
#include "commands/autonomous/autonomous_L1_FE.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <units/length.h>

#include "commands/autonomous/auto_utils.h"
#include "commands/drive_choreo.h"
#include "commands/go_to_position_command.h"

AutonomousL1FE::AutonomousL1FE(ElevatorSubsystem& elevator,
                               IntakeSubsystem& intake,
                               SwerveDriveSubsystem& swerve,
                               VisionSubsystem& vision)
    : m_Elevator{elevator}
    , m_Intake{intake}
    , m_Swerve{swerve}
    , m_Vision{vision}
    , m_armPositionEventCallback{[this](ArmPosition position) {
      auto_utils::SetAutoArmPosition(position, &m_Elevator, &m_Intake);
    }}
    , m_allCommands{
          frc2::SequentialCommandGroup{DriveChoreo{m_Swerve, "L1_FE", true, m_armPositionEventCallback},
                                       frc2::InstantCommand([this]() { m_Intake.Outtake(0.15); }, {&m_Intake}),
                                       frc2::WaitCommand(300_ms),
                                       GoToPositionCommand(&m_Elevator, setpoints::stow)}} {}

// Called when the command is initially scheduled.
void AutonomousL1FE::Initialize() {
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousL1FE::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousL1FE::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousL1FE::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousL1FE::GetName() const {
  return "04. L1 FE";
}

frc2::Command* AutonomousL1FE::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
