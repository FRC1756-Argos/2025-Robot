/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/intake_subsystem.h"

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/motors.h"

IntakeSubsystem::IntakeSubsystem(argos_lib::RobotInstance robotInstance)
    : m_intakeMotor(
          GetCANAddr(address::comp_bot::intake::intakeMotor, address::practice_bot::intake::intakeMotor, robotInstance))
    , m_robotInstance(robotInstance) {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::intake::intake,
                                         motorConfig::practice_bot::intake::intake>(
      m_intakeMotor, 100_ms, robotInstance);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}
void IntakeSubsystem::Disable() {
  Stop();
}
void IntakeSubsystem::Intake(double speed) {
  m_intakeMotor.Set(speed);
}
void IntakeSubsystem::Outtake(double speed) {
  m_intakeMotor.Set(speed);
}
void IntakeSubsystem::Stop() {
  m_intakeMotor.Set(0.0);
}
