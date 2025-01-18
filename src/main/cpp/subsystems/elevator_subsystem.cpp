/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/motors.h"
#include "subsystems/elevator_subsystem.h"

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
    , m_robotInstance(robotInstance) {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::elevator::elevator,
                                         motorConfig::practice_bot::elevator::elevator>(
      m_elevatorPrimary, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::elevator::elevator,
                                         motorConfig::practice_bot::elevator::elevator>(
      m_elevatorSecondary, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::elevator::arm,
                                         motorConfig::practice_bot::elevator::arm>(m_armMotor, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::elevator::wrist,
                                         motorConfig::practice_bot::elevator::wrist>(
      m_wristMotor, 100_ms, robotInstance);
}

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {}
