/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/intake_subsystem.h"

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/motors.h"
#include "frc/smartdashboard/SmartDashboard.h"

IntakeSubsystem::IntakeSubsystem(argos_lib::RobotInstance robotInstance)
    : m_intakeMotor(
          GetCANAddr(address::comp_bot::intake::intakeMotor, address::practice_bot::intake::intakeMotor, robotInstance))
    , m_robotInstance(robotInstance) {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::intake::intake,
                                         motorConfig::practice_bot::intake::intake>(
      m_intakeMotor, 100_ms, robotInstance);
  m_haveCoral = false, m_haveAlgae = false;
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Intake Motor Current", m_intakeMotor.GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber("Intake Motor TPS", m_intakeMotor.GetVelocity().GetValue().value());
  frc::SmartDashboard::PutNumber("IsCoralDetected", IsCoralDetected());
  frc::SmartDashboard::PutNumber("IsAlgaeDetected", IsAlgaeDetected());
  frc::SmartDashboard::PutNumber("IsAlgaeDetected", IsAlgaeLost());
  frc::SmartDashboard::PutNumber("haveCoral", m_haveCoral);
  frc::SmartDashboard::PutNumber("haveAlgae", m_haveAlgae);
}

void IntakeSubsystem::Disable() {
  Stop();
}

void IntakeSubsystem::IntakeCoral(double speed) {
  m_intakeMotor.Set(std::abs(speed));
  m_haveCoral = true;
  m_haveAlgae = false;
}

void IntakeSubsystem::OuttakeCoral(double speed) {
  m_intakeMotor.Set(-std::abs(speed));
  m_haveCoral = false;
  m_haveAlgae = false;
}

void IntakeSubsystem::IntakeAlgae(double speed) {
  m_intakeMotor.Set(std::abs(speed));
  m_haveAlgae = true;
  m_haveCoral = false;
}

void IntakeSubsystem::OuttakeAlgae(double speed) {
  m_intakeMotor.Set(-std::abs(speed));
  m_haveAlgae = false;
  m_haveCoral = false;
}

bool IntakeSubsystem::IsCoralDetected() {
  return m_intakeMotor.GetVelocity().GetValue() > 30.0_tps && m_intakeMotor.GetStatorCurrent().GetValue() > 12_A;
}

bool IntakeSubsystem::IsAlgaeDetected() {
  // Turns per second is negative when intaking algae.  Ensure the value is negative (with some wiggle room), but also not moving, while motor current is also elevated.
  return 1_tps > m_intakeMotor.GetVelocity().GetValue() && m_intakeMotor.GetVelocity().GetValue() > -10.0_tps &&
         m_intakeMotor.GetStatorCurrent().GetValue() >
             15_A;  // Need confirmation on the amperage threshold, is untested.
}

bool IntakeSubsystem::IsAlgaeLost() {
  return !m_haveCoral && m_haveAlgae && !IsAlgaeDetected();
}

void IntakeSubsystem::Stop() {
  m_intakeMotor.Set(0.0);
}
