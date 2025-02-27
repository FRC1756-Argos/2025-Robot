/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/measure_up.h"
#include "constants/motors.h"
#include "frc/smartdashboard/SmartDashboard.h"

ClimberSubsystem::ClimberSubsystem(argos_lib::RobotInstance robotInstance)
    : m_climberWinch(GetCANAddr(
          address::comp_bot::climber::climberPrimary, address::practice_bot::climber::climberPrimary, robotInstance))
    , m_climberPositionMotor(GetCANAddr(address::comp_bot::climber::climberSecondary,
                                    address::practice_bot::climber::climberSecondary,
                                    robotInstance))
    , m_robotInstance(robotInstance) {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::climberPrimary,
                                         motorConfig::practice_bot::climber::climberPrimary>(
      m_climberWinch, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::climberSecondary,
                                         motorConfig::practice_bot::climber::climberSecondary>(
      m_climberPositionMotor, 100_ms, robotInstance);
  EnableSoftLimits();
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
void ClimberSubsystem::Disable() {
  ClimberStop();
}
void ClimberSubsystem::ClimberUp(double speed) {
  speed = std::abs(speed);
  if (GetClimberManualOverride()) {
    m_climberPositionMotor.Set(speed);
  }
}
void ClimberSubsystem::ClimberDown(double speed) {
  speed = -std::abs(speed);
  if (GetClimberManualOverride()) {
    m_climberPositionMotor.Set(speed);
  }
}
void ClimberSubsystem::WinchIn(double speed) {
  if (!GetClimberManualOverride()) {
    SetPrimaryBreakModeToBreak(false);
    PositionMotorStop();
    m_climberWinch.Set(speed);
  }
}
void ClimberSubsystem::WinchStop(){
  SetPrimaryBreakModeToBreak(true);
  ClimberStop();
}

void ClimberSubsystem::SetPrimaryBreakModeToBreak(bool value){
  if(value){
    m_climberWinch.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  }
  else{
    m_climberWinch.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
  }
}

void ClimberSubsystem::ClimberStop() {
  m_climberWinch.Set(0.0);
  m_climberPositionMotor.Set(0.0);
}

void ClimberSubsystem::PositionMotorStop(){
  m_climberPositionMotor.Set(0.0);
}

void ClimberSubsystem::SetClimberManualOverride(bool desiredOverrideState) {
  m_climberManualOverride = desiredOverrideState;
}
bool ClimberSubsystem::GetClimberManualOverride() const {
  return m_climberManualOverride;
}

void ClimberSubsystem::ClimberMoveToAngle(units::degree_t angle) {
  SetClimberManualOverride(false);

  angle = std::clamp<units::degree_t>(angle, measure_up::climber::minAngle, measure_up::climber::maxAngle);
  m_climberPositionMotor.SetControl(
      ctre::phoenix6::controls::MotionMagicExpoVoltage(sensor_conversions::climber::ToSensorUnit(angle)));
}

units::degree_t ClimberSubsystem::ClimberGetAngle() {
  //frc::SmartDashboard::PutNumber("Cimber Angle", ClimberGetAngle().value());
  //frc::SmartDashboard::PutNumber("Cimber Motor Position", m_climberPositionMotor.GetPosition().GetValue().value());
  return sensor_conversions::climber::ToAngle(m_climberPositionMotor.GetPosition().GetValue());
}

bool ClimberSubsystem::ClimberIsAtSetPoint() {
  if (m_climberPositionMotor.GetControlMode().GetValue() !=
          ctre::phoenix6::signals::ControlModeValue::MotionMagicExpoVoltage &&
      m_climberPositionMotor.GetControlMode().GetValue() !=
          ctre::phoenix6::signals::ControlModeValue::MotionMagicExpoVoltageFOC) {
    return false;
  }
  return units::math::abs(sensor_conversions::climber::ToAngle(
             units::turn_t{m_climberPositionMotor.GetClosedLoopError().GetValue()})) < 0.5_deg;
}

void ClimberSubsystem::EnableSoftLimits() {
  ctre::phoenix6::configs::SoftwareLimitSwitchConfigs SoftLimits;
  SoftLimits.ForwardSoftLimitThreshold = sensor_conversions::climber::ToSensorUnit(measure_up::climber::maxAngle);
  SoftLimits.ReverseSoftLimitThreshold = sensor_conversions::climber::ToSensorUnit(measure_up::climber::minAngle);
  SoftLimits.ForwardSoftLimitEnable = true;
  SoftLimits.ReverseSoftLimitEnable = true;
  m_climberPositionMotor.GetConfigurator().Apply(SoftLimits);
}
void ClimberSubsystem::DisableSoftLimits() {
  ctre::phoenix6::configs::SoftwareLimitSwitchConfigs SoftLimits;
  SoftLimits.ForwardSoftLimitEnable = false;
  SoftLimits.ReverseSoftLimitEnable = false;
  m_climberPositionMotor.GetConfigurator().Apply(SoftLimits);
}
