/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "addresses.h"
#include "argos_lib/config/status_frame_config.h"
#include "control_loops.h"
#include "units/current.h"
#include "units/time.h"
#include "units/voltage.h"
#include "utils/sensor_conversions.h"

namespace motorConfig {
  //////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Motor configuration settings shared by all robot configurations
  //////////////////////////////////////////////////////////////////////////////////////////////////
  namespace common {
    constexpr static auto neutralDeadband = 0.001;
  }  // namespace common

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Motor configurations specific to competition robot
  //////////////////////////////////////////////////////////////////////////////////////////////////
  namespace comp_bot {
    namespace drive {
      struct genericDrive {
        constexpr static auto inverted = false;
        constexpr static auto statorCurrentLimit = 80_A;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::drive::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::drive::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::drive::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::drive::drive::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::drive::drive::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::drive::drive::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::drive::drive::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::drive::drive::gravityType;
      };

      struct frontLeftTurn {
        constexpr static auto inverted = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto selectedSensor_addr = address::comp_bot::encoders::frontLeftEncoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        constexpr static auto rotorToSensorRatio = sensor_conversions::swerve_drive::turn::turnGearRatio;
        constexpr static auto sensorToMechanismRatio = sensor_conversions::swerve_drive::turn::sensorConversionFactor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::drive::rotate::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::drive::rotate::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::drive::rotate::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::drive::rotate::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::drive::rotate::gravityType;
      };
      struct frontRightTurn {
        constexpr static auto inverted = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto selectedSensor_addr = address::comp_bot::encoders::frontRightEncoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        constexpr static auto rotorToSensorRatio = sensor_conversions::swerve_drive::turn::turnGearRatio;
        constexpr static auto sensorToMechanismRatio = sensor_conversions::swerve_drive::turn::sensorConversionFactor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::drive::rotate::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::drive::rotate::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::drive::rotate::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::drive::rotate::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::drive::rotate::gravityType;
      };
      struct backRightTurn {
        constexpr static auto inverted = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto selectedSensor_addr = address::comp_bot::encoders::backRightEncoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        constexpr static auto rotorToSensorRatio = sensor_conversions::swerve_drive::turn::turnGearRatio;
        constexpr static auto sensorToMechanismRatio = sensor_conversions::swerve_drive::turn::sensorConversionFactor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::drive::rotate::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::drive::rotate::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::drive::rotate::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::drive::rotate::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::drive::rotate::gravityType;
      };
      struct backLeftTurn {
        constexpr static auto inverted = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto selectedSensor_addr = address::comp_bot::encoders::backLeftEncoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        constexpr static auto rotorToSensorRatio = sensor_conversions::swerve_drive::turn::turnGearRatio;
        constexpr static auto sensorToMechanismRatio = sensor_conversions::swerve_drive::turn::sensorConversionFactor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::drive::rotate::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::drive::rotate::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::drive::rotate::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::drive::rotate::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::drive::rotate::gravityType;
      };
    }  // namespace drive
    namespace elevator {
      struct primaryElevator {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statorCurrentLimit = 40_A;
        constexpr static auto pid0_kP = controlLoop::comp_bot::elevator::elevator::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::elevator::elevator::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::elevator::elevator::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::elevator::elevator::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::elevator::elevator::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::elevator::elevator::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::elevator::elevator::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::elevator::elevator::gravityType;
        constexpr static auto rotorToSensorRatio = 5.0;  // 5:1 motor
        constexpr static auto sensorToMechanismRatio = 1.0;
      };
      struct secondaryElevator {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
      };

      struct arm {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto dutyCycleNeutralDeadband = 0.02;
        constexpr static auto statorCurrentLimit = 80_A;
        constexpr static auto selectedSensor_addr = address::comp_bot::encoders::armEncoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        constexpr static auto pid0_kP = controlLoop::comp_bot::elevator::arm::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::elevator::arm::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::elevator::arm::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::elevator::arm::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::elevator::arm::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::elevator::arm::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::elevator::arm::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::elevator::arm::gravityType;
        constexpr static auto motionMagic_cruiseVelocity =
            controlLoop::comp_bot::elevator::arm::motionMagic_cruiseVelocity;
        constexpr static auto motionMagic_acceleration = controlLoop::comp_bot::elevator::arm::motionMagic_acceleration;
        constexpr static auto motionMagic_jerk = controlLoop::comp_bot::elevator::arm::motionMagic_jerk;
        constexpr static auto motionMagic_expo_kV = controlLoop::comp_bot::elevator::arm::motionMagic_expo_kV;
        constexpr static auto motionMagic_expo_kA = controlLoop::comp_bot::elevator::arm::motionMagic_expo_kA;
        constexpr static auto rotorToSensorRatio = 108.0;  // 9:1 motor + 12:1 gearbox
        constexpr static auto sensorToMechanismRatio = 1.0;
      };
      struct wrist {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statorCurrentLimit = 20_A;
        constexpr static auto selectedSensor_addr = address::comp_bot::encoders::wristEncoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        constexpr static auto pid0_kP = controlLoop::comp_bot::elevator::wrist::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::elevator::wrist::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::elevator::wrist::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::elevator::wrist::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::elevator::wrist::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::elevator::wrist::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::elevator::wrist::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::elevator::wrist::gravityType;
        constexpr static auto motionMagic_cruiseVelocity =
            controlLoop::comp_bot::elevator::wrist::motionMagic_cruiseVelocity;
        constexpr static auto motionMagic_acceleration =
            controlLoop::comp_bot::elevator::wrist::motionMagic_acceleration;
        constexpr static auto motionMagic_jerk = controlLoop::comp_bot::elevator::wrist::motionMagic_jerk;
        constexpr static auto motionMagic_expo_kV = controlLoop::comp_bot::elevator::wrist::motionMagic_expo_kV;
        constexpr static auto motionMagic_expo_kA = controlLoop::comp_bot::elevator::wrist::motionMagic_expo_kA;
        constexpr static auto rotorToSensorRatio = 25.0;  // /@todo need motor ratio
        constexpr static auto sensorToMechanismRatio = 1.0;
      };
    }  // namespace elevator
    namespace climber {
      struct climberPrimary {
        constexpr static auto inverted = true;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statorCurrentLimit = 80_A;
        constexpr static auto pid0_kP = controlLoop::comp_bot::climber::climber::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::climber::climber::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::climber::climber::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::climber::climber::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::climber::climber::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::climber::climber::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::climber::climber::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::climber::climber::gravityType;
      };
      struct climberSecondary {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statorCurrentLimit = 60_A;
        constexpr static auto pid0_kP = controlLoop::comp_bot::climber::climber::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::climber::climber::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::climber::climber::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::climber::climber::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::climber::climber::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::climber::climber::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::climber::climber::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::climber::climber::gravityType;
        constexpr static auto motionMagic_cruiseVelocity =
            controlLoop::comp_bot::elevator::wrist::motionMagic_cruiseVelocity;
        constexpr static auto motionMagic_acceleration =
            controlLoop::comp_bot::elevator::wrist::motionMagic_acceleration;
        constexpr static auto motionMagic_jerk = controlLoop::comp_bot::elevator::wrist::motionMagic_jerk;
        constexpr static auto motionMagic_expo_kV = controlLoop::comp_bot::elevator::wrist::motionMagic_expo_kV;
        constexpr static auto motionMagic_expo_kA = controlLoop::comp_bot::elevator::wrist::motionMagic_expo_kA;
        constexpr static auto selectedSensor_addr = address::comp_bot::encoders::climberEncoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        constexpr static auto rotorToSensorRatio = 4.0 * 4.0 * 60.0 / 30.0;
        constexpr static auto sensorToMechanismRatio = 1.0;
      };
    }  // namespace climber
    namespace intake {
      struct intake {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statorCurrentLimit = 65_A;
        constexpr static auto voltageOpenLoopRampPeriod = 200_ms;
        constexpr static auto dutyCycleOpenLoopRampPeriod = 200_ms;
        constexpr static auto pid0_kP = controlLoop::comp_bot::intake::intake::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::intake::intake::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::intake::intake::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::intake::intake::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::intake::intake::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::intake::intake::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::intake::intake::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::intake::intake::gravityType;
      };

    }  // namespace intake
  }  // namespace comp_bot

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Motor configurations specific to practice robot
  //////////////////////////////////////////////////////////////////////////////////////////////////
  namespace practice_bot {
    namespace drive {
      using genericDrive = motorConfig::comp_bot::drive::genericDrive;

      using frontLeftTurn = motorConfig::comp_bot::drive::frontLeftTurn;
      using frontRightTurn = motorConfig::comp_bot::drive::frontRightTurn;
      using backRightTurn = motorConfig::comp_bot::drive::backRightTurn;
      using backLeftTurn = motorConfig::comp_bot::drive::backLeftTurn;
    }  // namespace drive
    namespace elevator {
      using primaryElevator = motorConfig::comp_bot::elevator::primaryElevator;
      using secondaryElevator = motorConfig::comp_bot::elevator::secondaryElevator;
      using arm = motorConfig::comp_bot::elevator::arm;
      using wrist = motorConfig::comp_bot::elevator::wrist;
    }  // namespace elevator
    namespace climber {
      using climberPrimary = motorConfig::comp_bot::climber::climberPrimary;
      using climberSecondary = motorConfig::comp_bot::climber::climberSecondary;
    }  // namespace climber
    namespace intake {
      using intake = motorConfig::comp_bot::intake::intake;
    }  // namespace intake
  }  // namespace practice_bot
}  // namespace motorConfig
