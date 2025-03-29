/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <argos_lib/config/config_types.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "constants/position.h"

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

  void Disable();

  void ElevatorMoveToHeight(units::inch_t height);

  void ArmMoveToAngle(units::degree_t armAngle);

  void SetWristAngle(units::degree_t wristAngle);

  [[nodiscard]] frc2::CommandPtr CommandElevatorToHeight(units::inch_t height);

  [[nodiscard]] frc2::CommandPtr CommandArmToAngle(units::degree_t armAngle);

  [[nodiscard]] frc2::CommandPtr CommandWristToAngle(units::degree_t wristAngle);

  void SetElevatorManualOverride(bool desiredOverrideState);

  [[nodiscard]] bool GetElevatorManualOverride() const;

  [[nodiscard]] units::inch_t GetElevatorHeight();

  [[nodiscard]] bool IsElevatorAtSetPoint();

  [[nodiscard]] units::degree_t GetArmAngle();

  [[nodiscard]] bool IsArmAtSetPoint();

  [[nodiscard]] units::degree_t GetWristAngle();

  [[nodiscard]] bool IsWristAtSetPoint();

  [[nodiscard]] Position GetPosition();

  [[nodiscard]] bool IsAtSetPoint();

  [[nodiscard]] Position GetSetpoint();

  void GoToPosition(const Position target);

  [[nodiscard]] frc2::CommandPtr CommandToPosition(const Position target);

  [[nodiscard]] bool IsAtStowPosition();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX m_elevatorPrimary;
  ctre::phoenix6::hardware::TalonFX m_elevatorSecondary;
  ctre::phoenix6::hardware::TalonFX m_armMotor;
  ctre::phoenix6::hardware::TalonFX m_wristMotor;
  argos_lib::RobotInstance m_robotInstance;
  bool m_elevatorManualOverride;
  bool m_elevatorHomed;
  bool m_armHomed;
  bool m_wristHomed;
  void EnableElevatorSoftLimits();
  void DisableElevatorSoftLimits();
  void EnableArmSoftLimits();
  void DisableArmSoftLimits();
  void EnableWristSoftLimits();
  void DisableWristSoftLimits();
};
