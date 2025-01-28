/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <argos_lib/config/config_types.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  explicit ElevatorSubsystem(argos_lib::RobotInstance robotInstance);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void ElevatorMove(double speed);

  void Pivot(double speed);

  void Rotate(double speed);

  void SetWristAngle(units::degree_t wristAngle);

  void Disable();

  void SetElevatorManualOverride(bool desiredOverrideState);

  [[nodiscard]] bool GetElevatorManualOverride() const;

  [[nodiscard]] units::degree_t GetWristAngle();

  [[nodiscard]] bool IsWristAtSetPoint();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX m_elevatorPrimary;
  ctre::phoenix6::hardware::TalonFX m_elevatorSecondary;
  ctre::phoenix6::hardware::TalonFX m_armMotor;
  ctre::phoenix6::hardware::TalonFX m_wristMotor;
  argos_lib::RobotInstance m_robotInstance;
  bool m_elevatorManualOverride;
  bool m_wristHomed;
  void EnableWristSoftLimits();
  void DisableWristSoftLimits();
};
