/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <chrono>

#include "subsystems/elevator_subsystem.h"
#include "subsystems/intake_subsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class L4CoralPlacementCommand : public frc2::CommandHelper<frc2::Command, L4CoralPlacementCommand> {
 public:
  L4CoralPlacementCommand(ElevatorSubsystem* elevatorSubsystem, IntakeSubsystem* intakeSubsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ElevatorSubsystem* m_pElevatorSubsystem;
  IntakeSubsystem* m_pIntakeSubsystem;
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;
  bool m_placed;
};
