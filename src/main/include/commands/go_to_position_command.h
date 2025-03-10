/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>

#include <memory>
#include <vector>

#include "constants/position.h"
#include "subsystems/elevator_subsystem.h"

class GoToPositionCommand : public frc2::CommandHelper<frc2::Command, GoToPositionCommand> {
 public:
  GoToPositionCommand(ElevatorSubsystem* elevatorSubsystem, Position m_position, bool coralMode = true);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ElevatorSubsystem* m_pElevatorSubsystem;
  Position m_position;
  const bool m_coralMode;

  [[nodiscard]] units::degree_t GetSafeArmTarget(units::degree_t target);
  std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
};
