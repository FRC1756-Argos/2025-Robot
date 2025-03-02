/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_L4_G.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <units/length.h>

#include "commands/autonomous/auto_utils.h"
#include "commands/drive_choreo.h"
#include "commands/go_to_position_command.h"
#include "commands/l4_coral_placement_command.h"

AutonomousL4G::AutonomousL4G(ElevatorSubsystem& elevator,
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
          frc2::SequentialCommandGroup{
            frc2::WaitCommand(300_ms),
            frc2::InstantCommand([this]() { m_Vision.SetLeftAlign(true); }, {&m_Vision}),
            frc2::WaitCommand(6000_ms),
            frc2::InstantCommand([this]() { m_Vision.SetLeftAlign(false); }, {&m_Vision}),
            GoToPositionCommand(&m_Elevator, setpoints::levelFourRight),
            frc2::WaitCommand(2000_ms),
            L4CoralPlacementCommand(&m_Elevator, &m_Intake),
            frc2::WaitCommand(2000_ms),
            frc2::InstantCommand([this]() { m_Intake.Outtake(0.3); }, {&m_Intake}),
            GoToPositionCommand(&m_Elevator, setpoints::stow),
            frc2::InstantCommand([this]() { m_Intake.Stop(); }, {&m_Intake})}} {}

// Called when the command is initially scheduled.
void AutonomousL4G::Initialize() {
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousL4G::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousL4G::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousL4G::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousL4G::GetName() const {
  return "5. L4 G";
}

frc2::Command* AutonomousL4G::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
