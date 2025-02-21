/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/measure_up.h"
#include "constants/motors.h"

ClimberSubsystem::ClimberSubsystem(argos_lib::RobotInstance robotInstance)
    : m_climberPrimary(GetCANAddr(
          address::comp_bot::climber::climberPrimary, address::practice_bot::climber::climberPrimary, robotInstance))
    , m_climberSecondary(GetCANAddr(address::comp_bot::climber::climberSecondary,
                                    address::practice_bot::climber::climberSecondary,
                                    robotInstance))
    , m_robotInstance(robotInstance) {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::climberPrimary,
                                         motorConfig::practice_bot::climber::climberPrimary>(
      m_climberPrimary, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::climberSecondary,
                                         motorConfig::practice_bot::climber::climberSecondary>(
      m_climberSecondary, 100_ms, robotInstance);
  EnableSoftLimits();
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
void ClimberSubsystem::Disable() {
  Stop();
}
void ClimberSubsystem::Move(double speed) {
  if (GetClimberManualOverride()) {
    m_climberPrimary.Set(speed);
  }
}
void ClimberSubsystem::Stop() {
  m_climberPrimary.Set(0.0);
}
void ClimberSubsystem::SetClimberManualOverride(bool desiredOverrideState) {
  m_climberManualOverride = desiredOverrideState;
}
bool ClimberSubsystem::GetClimberManualOverride() const {
  return m_climberManualOverride;
}

void ClimberSubsystem::MoveToAngle(units::degree_t angle) {
  SetClimberManualOverride(false);
  angle = std::clamp<units::degree_t>(angle, measure_up::climber::minAngle, measure_up::climber::maxAngle);
  m_climberSecondary.SetControl(
      ctre::phoenix6::controls::MotionMagicExpoVoltage(sensor_conversions::climber::ToSensorUnit(angle)));
}
units::degree_t ClimberSubsystem::GetAngle() {
  return sensor_conversions::climber::ToAngle(m_climberSecondary.GetPosition().GetValue());
}

bool ClimberSubsystem::IsAtSetPoint() {
  if (m_climberSecondary.GetControlMode().GetValue() !=
          ctre::phoenix6::signals::ControlModeValue::MotionMagicExpoVoltage &&
      m_climberSecondary.GetControlMode().GetValue() !=
          ctre::phoenix6::signals::ControlModeValue::MotionMagicExpoVoltageFOC) {
    return false;
  }
  return units::math::abs(sensor_conversions::climber::ToAngle(
             units::turn_t{m_climberSecondary.GetClosedLoopError().GetValue()})) < 0.5_deg;
}
void ClimberSubsystem::EnableSoftLimits() {
  ctre::phoenix6::configs::SoftwareLimitSwitchConfigs SoftLimits;
  SoftLimits.ForwardSoftLimitThreshold = sensor_conversions::climber::ToSensorUnit(measure_up::climber::maxAngle);
  SoftLimits.ReverseSoftLimitThreshold = sensor_conversions::climber::ToSensorUnit(measure_up::climber::minAngle);
  SoftLimits.ForwardSoftLimitEnable = true;
  SoftLimits.ReverseSoftLimitEnable = true;
  m_climberSecondary.GetConfigurator().Apply(SoftLimits);
}
void ClimberSubsystem::DisableSoftLimits() {
  ctre::phoenix6::configs::SoftwareLimitSwitchConfigs SoftLimits;
  SoftLimits.ForwardSoftLimitEnable = false;
  SoftLimits.ReverseSoftLimitEnable = false;
  m_climberSecondary.GetConfigurator().Apply(SoftLimits);
}
