/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <argos_lib/config/config_types.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  explicit IntakeSubsystem(argos_lib::RobotInstance RobotInstance);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void Disable();
  void Intake();
  void Outtake();
  void Stop();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX m_intakeMotor;
  argos_lib::RobotInstance m_robotInstance;
};
