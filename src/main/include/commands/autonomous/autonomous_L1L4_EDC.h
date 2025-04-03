/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <string>

#include "commands/autonomous/autonomous_command.h"
#include "commands/drive_by_time_vision.h"
#include "commands/go_to_position_command.h"
#include "commands/l4_coral_placement_command.h"
#include "subsystems/elevator_subsystem.h"
#include "subsystems/intake_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"
#include "subsystems/vision_subsystem.h"

class AutonomousL1L4EDC
    : public frc2::CommandHelper<frc2::Command, AutonomousL1L4EDC>
    , public AutonomousCommand {
 public:
  AutonomousL1L4EDC(ElevatorSubsystem& elevator,
                    IntakeSubsystem& intake,
                    SwerveDriveSubsystem& swerve,
                    VisionSubsystem& vision);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  /**
   * @copydoc AutonomousCommand::GetName()
   */
  std::string GetName() const final;
  /**
   * @copydoc AutonomousCommand::GetCommand()
   */
  frc2::Command* GetCommand() final;

 private:
  ElevatorSubsystem& m_Elevator;
  IntakeSubsystem& m_Intake;
  SwerveDriveSubsystem& m_Swerve;
  VisionSubsystem& m_Vision;
  std::function<void(ArmPosition)> m_armPositionEventCallback;

  frc2::SequentialCommandGroup m_allCommands;
};
