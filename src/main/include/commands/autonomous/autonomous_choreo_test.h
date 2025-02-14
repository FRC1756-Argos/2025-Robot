/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <string>

#include "commands/autonomous/autonomous_command.h"
#include "subsystems/elevator_subsystem.h"
#include "subsystems/intake_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"
#include "subsystems/vision_subsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutonomousChoreoTest : public frc2::CommandHelper<frc2::Command, autonomous_choreo_test> {
 public:
  AutonomousChoreoTest(ElevatorSubsystem& elevator,
                       IntakeSubsystem& intake,
                       SwerveDriveSubsystem& swerve,
                       VisionSubsystem& vision);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ElevatorSubsystem& m_Elevator;
  IntakeSubsystem& m_Intake;
  SwerveDriveSubsystem& m_Swerve;
  VisionSubsystem& m_Vision;

  frc2::SequentialCommandGroup m_allCommands;
};
