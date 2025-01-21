/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
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
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
void ClimberSubsystem::Disable() {
  Stop();
}
void ClimberSubsystem::Up() {
  m_climberPrimary.Set(1.0);
  m_climberSecondary.Set(-1.0);
}
void ClimberSubsystem::Down() {
  m_climberPrimary.Set(-1.0);
  m_climberSecondary.Set(1.0);
}
void ClimberSubsystem::Stop() {
  m_climberPrimary.Set(0.0);
  m_climberSecondary.Set(0.0);
}
