/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_L4_JL.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <units/length.h>

#include "commands/autonomous/auto_utils.h"
#include "commands/drive_choreo.h"
#include "commands/go_to_position_command.h"

AutonomousL4JL::AutonomousL4JL(ElevatorSubsystem& elevator,
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
          frc2::SequentialCommandGroup{DriveChoreo{m_Swerve, "L4Place_1", true, m_armPositionEventCallback},
                                       frc2::InstantCommand([this]() { m_Vision.SetRightAlign(true); }, {&m_Vision}),
                                       DriveByTimeVisionCommand(m_Swerve, m_Vision, false, 1000_ms),
                                       GoToPositionCommand(&m_Elevator, setpoints::levelFourRight),
                                       frc2::WaitCommand(200_ms),
                                       L4CoralPlacementCommand(&m_Elevator, &m_Intake),
                                       DriveChoreo{m_Swerve, "L4Place_2", false, m_armPositionEventCallback},
                                       GoToPositionCommand(&m_Elevator, setpoints::coralStationLeft),
                                       frc2::WaitCommand(500_ms),
                                       DriveChoreo{m_Swerve, "L4Place_3", false, m_armPositionEventCallback},
                                       frc2::InstantCommand([this]() { m_Vision.SetRightAlign(true); }, {&m_Vision}),
                                       DriveByTimeVisionCommand(m_Swerve, m_Vision, false, 750_ms),
                                       GoToPositionCommand(&m_Elevator, setpoints::levelFourRight),
                                       frc2::WaitCommand(200_ms),
                                       L4CoralPlacementCommand(&m_Elevator, &m_Intake),
                                       GoToPositionCommand(&m_Elevator, setpoints::stow)}} {}

// Called when the command is initially scheduled.
void AutonomousL4JL::Initialize() {
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousL4JL::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousL4JL::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousL4JL::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousL4JL::GetName() const {
  return "06. L4 JL";
}

frc2::Command* AutonomousL4JL::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
