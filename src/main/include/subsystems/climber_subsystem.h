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
   */
  void Periodic() override;
  void Disable();
  void ClimberUp(double speed = 0.1);
  void ClimberDown(double speed = 0.1);
  void WinchIn(double speed = 0.75, bool stopPositionMotor = true);
  void ClimberStop();
  void PositionMotorStop();
  units::ampere_t GetPositionMotorCurrent();
  void SetClimberManualOverride(bool desiredOverrideState);
  [[nodiscard]] bool GetClimberManualOverride() const;

  void ClimberMoveToAngle(units::degree_t angle);

  [[nodiscard]] units::degree_t ClimberGetAngle();

  [[nodiscard]] bool ClimberIsAtSetPoint();

  void SetPrimaryBreakModeToBreak(bool value);
  void SetPositionMotorBreakModeToBreak(bool value);

  void WinchStop();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX m_climberWinch;
  ctre::phoenix6::hardware::TalonFX m_climberPositionMotor;
  argos_lib::RobotInstance m_robotInstance;
  bool m_climberManualOverride;
  void EnableSoftLimits();
  void DisableSoftLimits();
};
