/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <memory>

#include "constants/position.h"
#include "subsystems/elevator_subsystem.h"

class GoToPositionCommand : public frc2::CommandHelper<frc2::Command, GoToPositionCommand> {
 public:
  GoToPositionCommand(ElevatorSubsystem* elevatorSubsystem, Position m_position);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ElevatorSubsystem* m_pElevatorSubsystem;
  Position m_position;
  std::unique_ptr<frc2::SequentialCommandGroup> m_movementSequence;
};
