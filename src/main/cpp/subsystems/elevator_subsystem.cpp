/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/elevator_subsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/feature_flags.h"
#include "constants/measure_up.h"
#include "constants/motors.h"
#include "utils/sensor_conversions.h"

ElevatorSubsystem::ElevatorSubsystem(argos_lib::RobotInstance robotInstance)
    : m_elevatorPrimary(GetCANAddr(address::comp_bot::elevator::elevatorPrimary,
                                   address::practice_bot::elevator::elevatorPrimary,
                                   robotInstance))
    , m_elevatorSecondary(GetCANAddr(address::comp_bot::elevator::elevatorSecondary,
                                     address::practice_bot::elevator::elevatorSecondary,
                                     robotInstance))
    , m_armMotor(
          GetCANAddr(address::comp_bot::elevator::armMotor, address::practice_bot::elevator::armMotor, robotInstance))
    , m_wristMotor(GetCANAddr(
          address::comp_bot::elevator::wristMotor, address::practice_bot::elevator::wristMotor, robotInstance))
    , m_robotInstance(robotInstance)
    , m_elevatorManualOverride(true)
    , m_armHomed(true) {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::elevator::primaryElevator,
                                         motorConfig::practice_bot::elevator::primaryElevator>(
      m_elevatorPrimary, 100_ms, robotInstance);

  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::elevator::secondaryElevator,
                                         motorConfig::practice_bot::elevator::secondaryElevator>(
      m_elevatorSecondary, 100_ms, robotInstance);

  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::elevator::arm,
                                         motorConfig::practice_bot::elevator::arm>(m_armMotor, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::elevator::wrist,
                                         motorConfig::practice_bot::elevator::wrist>(
      m_wristMotor, 100_ms, robotInstance);
  m_elevatorSecondary.SetControl(ctre::phoenix6::controls::Follower(m_elevatorPrimary.GetDeviceID(), true));
  EnableArmSoftLimits();
}

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {}

void ElevatorSubsystem::ElevatorMove(double speed) {
  if (GetElevatorManualOverride()) {
    m_elevatorPrimary.Set(speed);
  }
}

void ElevatorSubsystem::Pivot(double speed) {
  if (GetElevatorManualOverride()) {
    m_armMotor.Set(speed);
  }
}

void ElevatorSubsystem::ArmMoveToAngle(units::degree_t armAngle) {
  SetElevatorManualOverride(false);
  armAngle =
      std::clamp<units::degree_t>(armAngle, measure_up::elevator::arm::minAngle, measure_up::elevator::arm::maxAngle);
  m_armMotor.SetControl(
      ctre::phoenix6::controls::MotionMagicExpoVoltage(sensor_conversions::elevator::arm::ToSensorUnit(armAngle)));
}

void ElevatorSubsystem::Rotate(double speed) {
  if (GetElevatorManualOverride()) {
    m_wristMotor.Set(speed);
  }
}

void ElevatorSubsystem::Disable() {
  m_elevatorPrimary.Set(0.0);
  m_armMotor.Set(0.0);
  m_wristMotor.Set(0.0);
}

void ElevatorSubsystem::SetElevatorManualOverride(bool desiredOverrideState) {
  m_elevatorManualOverride = desiredOverrideState;
}

bool ElevatorSubsystem::GetElevatorManualOverride() const {
  return m_elevatorManualOverride;
}

units::degree_t ElevatorSubsystem::GetArmAngle() {
  return sensor_conversions::elevator::arm::ToAngle(m_armMotor.GetPosition().GetValue());
}

bool ElevatorSubsystem::IsArmAtSetPoint() {
  if constexpr (feature_flags::nt_debugging) {
    frc::SmartDashboard::PutString("ArmMode", m_armMotor.GetControlMode().GetValue().ToString());
    frc::SmartDashboard::PutNumber("ArmError", m_armMotor.GetClosedLoopError().GetValue());
    frc::SmartDashboard::PutNumber(
        "ArmAngleError",
        sensor_conversions::elevator::arm::ToAngle(units::degree_t{m_armMotor.GetClosedLoopError().GetValue()})
            .to<double>());
  }
  if (m_armMotor.GetControlMode().GetValue() != ctre::phoenix6::signals::ControlModeValue::PositionVoltage &&
      m_armMotor.GetControlMode().GetValue() != ctre::phoenix6::signals::ControlModeValue::PositionVoltageFOC) {
    return false;
  }
  return units::math::abs(sensor_conversions::elevator::arm::ToAngle(
             units::degree_t{m_armMotor.GetClosedLoopError().GetValue()})) < 1_deg;
}

void ElevatorSubsystem::EnableArmSoftLimits() {
  if (m_armHomed) {
    ctre::phoenix6::configs::SoftwareLimitSwitchConfigs ArmSoftLimits;
    ArmSoftLimits.ForwardSoftLimitThreshold =
        sensor_conversions::elevator::arm::ToSensorUnit(measure_up::elevator::arm::maxAngle);
    ArmSoftLimits.ReverseSoftLimitThreshold =
        sensor_conversions::elevator::arm::ToSensorUnit(measure_up::elevator::arm::minAngle);
    ArmSoftLimits.ForwardSoftLimitEnable = true;
    ArmSoftLimits.ReverseSoftLimitEnable = true;
    m_armMotor.GetConfigurator().Apply(ArmSoftLimits);
  }
}

void ElevatorSubsystem::DisableArmSoftLimits() {
  ctre::phoenix6::configs::SoftwareLimitSwitchConfigs ArmSoftLimits;
  ArmSoftLimits.ForwardSoftLimitEnable = false;
  ArmSoftLimits.ReverseSoftLimitEnable = false;
  m_armMotor.GetConfigurator().Apply(ArmSoftLimits);
}
