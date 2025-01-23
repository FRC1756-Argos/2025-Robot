/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  explicit ClimberSubsystem(argos_lib::RobotInstance robotInstance);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
  /*/
  void Periodic() override;
  void Disable();
  void Move(double speed);
  void Stop();
  void SetClimberManualOverride(bool desiredOverrideState);
  [[nodiscard]] bool GetClimberManualOverride() const;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX m_climberPrimary;
  ctre::phoenix6::hardware::TalonFX m_climberSecondary;
  argos_lib::RobotInstance m_robotInstance;
  bool m_climberManualOverride;
};
