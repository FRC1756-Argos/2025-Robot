/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/current.h>
#include <units/time.h>

#include <iostream>

#include <ctre/phoenix6/TalonFX.hpp>

#include "argos_lib/config/config_types.h"
#include "compile_time_member_check.h"
#include "status_frame_config.h"

namespace argos_lib {
  namespace falcon_config {

    HAS_MEMBER(forwardLimit_deviceID)
    HAS_MEMBER(forwardLimit_normalState)
    HAS_MEMBER(forwardLimit_source)
    HAS_MEMBER(inverted)
    HAS_MEMBER(neutralDeadband)
    HAS_MEMBER(neutralMode)
    HAS_MEMBER(dutyCycleNeutralDeadband)
    HAS_MEMBER(nominalOutputForward)
    HAS_MEMBER(nominalOutputReverse)
    HAS_MEMBER(peakOutputForward)
    HAS_MEMBER(peakOutputReverse)
    HAS_MEMBER(voltageOpenLoopRampPeriod)
    HAS_MEMBER(dutyCycleOpenLoopRampPeriod)
    HAS_MEMBER(pid0_kP)
    HAS_MEMBER(pid0_kI)
    HAS_MEMBER(pid0_kD)
    HAS_MEMBER(pid0_kS)
    HAS_MEMBER(pid0_kV)
    HAS_MEMBER(pid0_kA)
    HAS_MEMBER(pid0_kG)
    HAS_MEMBER(pid0_gravityType)
    HAS_MEMBER(pid1_kP)
    HAS_MEMBER(pid1_kI)
    HAS_MEMBER(pid1_kD)
    HAS_MEMBER(pid1_kS)
    HAS_MEMBER(pid1_kV)
    HAS_MEMBER(pid1_kA)
    HAS_MEMBER(pid1_gravityType)
    HAS_MEMBER(pid1_kG)
    HAS_MEMBER(motionMagic_cruiseVelocity)
    HAS_MEMBER(motionMagic_acceleration)
    HAS_MEMBER(motionMagic_jerk)
    HAS_MEMBER(motionMagic_expo_kV)
    HAS_MEMBER(motionMagic_expo_kA)
    HAS_MEMBER(reverseLimit_deviceID)
    HAS_MEMBER(reverseLimit_normalState)
    HAS_MEMBER(reverseLimit_source)
    HAS_MEMBER(rotorToSensorRatio)
    HAS_MEMBER(selectedSensor)
    HAS_MEMBER(selectedSensor_addr)
    HAS_MEMBER(sensorToMechanismRatio)
    HAS_MEMBER(statorCurrentLimit)
    HAS_MEMBER(statusFrameMotorMode)
    HAS_MEMBER(supplyCurrentLimit)
    HAS_MEMBER(supplyCurrentLowerLimit)
    HAS_MEMBER(supplyCurrentLowerTime)

    /**
     * @brief Configures a CTRE Falcon with only the fields provided.  All other fields
     *        are given the factory default values.
     *
     * @tparam T Structure containing any combination of the following members:
     *           - forwardLimit_deviceID
     *           - forwardLimit_normalState
     *           - forwardLimit_source
     *           - inverted
     *           - neutralDeadband
     *           - neutralMode
     *           - dutyCycleNeutralDeadband
     *           - peakOutputForward
     *           - peakOutputReverse
     *           - voltageOpenLoopRampPeriod
     *           - dutyCycleOpenLoopRampPeriod
     *           - pid0_kP
     *           - pid0_kI
     *           - pid0_kD
     *           - pid0_kS
     *           - pid0_kV
     *           - pid0_kA
     *           - pid0_kG
     *           - pid0_gravityType
     *           - pid1_kP
     *           - pid1_kI
     *           - pid1_kD
     *           - pid1_kS
     *           - pid1_kV
     *           - pid1_kA
     *           - pid1_kG
     *           - pid1_gravityType
     *           - motionMagic_cruiseVelocity
     *           - motionMagic_acceleration
     *           - motionMagic_jerk
     *           - motionMagic_expo_kV
     *           - motionMagic_expo_kA
     *           - reverseLimit_deviceID
     *           - reverseLimit_normalState
     *           - reverseLimit_source
     *           - rotorToSensorRatio
     *           - selectedSensor
     *           - selectedSensor_addr
     *           - sensorToMechanismRatio
     *           - statorCurrentLimit
     *           - statusFrameMotorMode
     *           - supplyCurrentLimit
     *           - supplyCurrentLowerLimit
     *           - supplyCurrentLowerTime
     * @param motorController Falcon object to configure
     * @param configTimeout Time to wait for response from Falcon
     * @return true Configuration succeeded
     * @return false Configuration failed
     */
    template <typename T>
    bool FalconConfig(ctre::phoenix6::hardware::TalonFX& motorController, units::millisecond_t configTimeout) {
      ctre::phoenix6::configs::TalonFXConfiguration config;

      if constexpr (has_inverted<T>{}) {
        config.MotorOutput.Inverted = T::inverted;
      }
      if constexpr (has_neutralMode<T>{}) {
        config.MotorOutput.NeutralMode = T::neutralMode;
      }
      if constexpr (has_dutyCycleNeutralDeadband<T>{}) {
        config.MotorOutput.DutyCycleNeutralDeadband = T::dutyCycleNeutralDeadband;
      }
      if constexpr (has_selectedSensor_addr<T>{}) {
        static_assert(has_selectedSensor<T>{} &&
                          T::selectedSensor != ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor,
                      "Remote sensor required when address provided");
        static_assert(T::selectedSensor_addr.address >= 0, "Remote sensor address must be non-negative");
        config.Feedback.FeedbackRemoteSensorID = T::selectedSensor_addr.address;
      }
      if constexpr (has_peakOutputForward<T>{}) {
        config.MotorOutput.PeakForwardDutyCycle = T::peakOutputForward;
      }
      if constexpr (has_peakOutputReverse<T>{}) {
        config.MotorOutput.PeakReverseDutyCycle = T::peakOutputReverse;
      }
      if constexpr (has_voltageOpenLoopRampPeriod<T>{}) {
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod =
            std::clamp<units::second_t>(T::voltageOpenLoopRampPeriod, 0_s, 1_s);
      }
      if constexpr (has_dutyCycleOpenLoopRampPeriod<T>{}) {
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
            std::clamp<units::second_t>(T::dutyCycleOpenLoopRampPeriod, 0_s, 1_s);
      }
      if constexpr (has_selectedSensor<T>{}) {
        config.Feedback.FeedbackSensorSource = T::selectedSensor;
        if constexpr (T::selectedSensor == ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder) {
          static_assert(has_sensorToMechanismRatio<T>{}, "Fused CANcoder mode requires sensor to mechanism ratio");
          static_assert(has_rotorToSensorRatio<T>{}, "Fused CANcoder mode requires rotor to sensor ratio");
          static_assert(T::sensorToMechanismRatio > 0, "sensorToMechanismRatio must be a positive value");
          static_assert(T::rotorToSensorRatio > 0, "rotorToSensorRatio must be a positive value");
          config.Feedback.SensorToMechanismRatio = T::sensorToMechanismRatio;
          config.Feedback.RotorToSensorRatio = T::rotorToSensorRatio;
        }
      }
      if constexpr (has_pid0_kP<T>{}) {
        config.Slot0.kP = T::pid0_kP;
      }
      if constexpr (has_pid0_kI<T>{}) {
        config.Slot0.kI = T::pid0_kI;
      }
      if constexpr (has_pid0_kD<T>{}) {
        config.Slot0.kD = T::pid0_kD;
      }
      if constexpr (has_pid0_kS<T>{}) {
        config.Slot0.kS = T::pid0_kS;
      }
      if constexpr (has_pid0_kV<T>{}) {
        config.Slot0.kV = T::pid0_kV;
      }
      if constexpr (has_pid0_kA<T>{}) {
        config.Slot0.kA = T::pid0_kA;
      }
      if constexpr (has_pid0_kG<T>{} && has_pid0_gravityType<T>{}) {
        config.Slot0.kG = T::pid0_kG;
        config.Slot0.GravityType = T::pid0_gravityType;
      }
      if constexpr (has_pid1_kP<T>{}) {
        config.Slot1.kP = T::pid1_kP;
      }
      if constexpr (has_pid1_kI<T>{}) {
        config.Slot1.kI = T::pid1_kI;
      }
      if constexpr (has_pid1_kD<T>{}) {
        config.Slot1.kD = T::pid1_kD;
      }
      if constexpr (has_pid1_kS<T>{}) {
        config.Slot1.kS = T::pid1_kS;
      }
      if constexpr (has_pid1_kV<T>{}) {
        config.Slot1.kV = T::pid1_kV;
      }
      if constexpr (has_pid1_kA<T>{}) {
        config.Slot1.kA = T::pid1_kA;
      }
      if constexpr (has_pid1_kG<T>{} && has_pid1_gravityType<T>{}) {
        config.Slot1.kG = T::pid1_kG;
        config.Slot1.GravityType = T::pid1_gravityType;
      }
      if constexpr (has_motionMagic_cruiseVelocity<T>{}) {
        config.MotionMagic.MotionMagicCruiseVelocity = T::motionMagic_cruiseVelocity;
      }
      if constexpr (has_motionMagic_acceleration<T>{}) {
        config.MotionMagic.MotionMagicAcceleration = T::motionMagic_acceleration;
      }
      if constexpr (has_motionMagic_jerk<T>{}) {
        config.MotionMagic.MotionMagicJerk = T::motionMagic_jerk;
      }
      if constexpr (has_motionMagic_expo_kV<T>{}) {
        config.MotionMagic.MotionMagicExpo_kV = T::motionMagic_expo_kV;
      }
      if constexpr (has_motionMagic_expo_kA<T>{}) {
        config.MotionMagic.MotionMagicExpo_kA = T::motionMagic_expo_kA;
      }
      if constexpr (has_supplyCurrentLimit<T>{} || has_supplyCurrentLowerLimit<T>{} ||
                    has_supplyCurrentLowerTime<T>{}) {
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        if constexpr (has_supplyCurrentLimit<T>{}) {
          constexpr units::ampere_t currentLimit = T::supplyCurrentLimit;
          static_assert(currentLimit.to<double>() > 0, "Supply current limit must be positive");
          config.CurrentLimits.SupplyCurrentLimit = currentLimit.to<double>();
        }
        if constexpr (has_supplyCurrentLowerLimit<T>{}) {
          constexpr units::ampere_t currentThreshold = T::supplyCurrentLowerLimit;
          static_assert(currentThreshold.to<double>() > 0, "Supply current threshold must be positive");
          config.CurrentLimits.SupplyCurrentLowerLimit = currentThreshold.to<double>();
        }
        if constexpr (has_supplyCurrentLowerTime<T>{}) {
          constexpr units::second_t currentThresholdTime = T::supplyCurrentLowerTime;
          static_assert(currentThresholdTime.to<double>() >= 0, "Supply current threshold time must be non-negative");
          static_assert(currentThresholdTime.to<double>() <= 1.275, "Current duration must be less than 1.275");
          config.CurrentLimits.SupplyCurrentLowerTime = currentThresholdTime.to<double>();
        }
      }
      if constexpr (has_statorCurrentLimit<T>{}) {
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        constexpr units::ampere_t currentLimit = T::statorCurrentLimit;
        static_assert(currentLimit.to<double>() > 0, "Stator current limit must be positive");
        config.CurrentLimits.StatorCurrentLimit = currentLimit;
      }
      if constexpr (has_forwardLimit_source<T>{} || has_forwardLimit_deviceID<T>{} ||
                    has_forwardLimit_normalState<T>{}) {
        if constexpr (has_forwardLimit_source<T>{}) {
          constexpr ctre::phoenix6::signals::ForwardLimitSourceValue source = T::forwardLimit_source;
          if constexpr (source != ctre::phoenix6::signals::ForwardLimitSourceValue::Disabled &&
                        source != ctre::phoenix6::signals::ForwardLimitSourceValue::LimitSwitchPin) {
            static_assert(has_forwardLimit_deviceID<T>{}, "Forward limit switch requires remote source device ID");
          }
          if constexpr (source != ctre::phoenix6::signals::ForwardLimitSourceValue::Disabled) {
            static_assert(has_forwardLimit_normalState<T>{},
                          "Forward limit switch configuration requires both source and normal state");
            config.HardwareLimitSwitch.ForwardLimitEnable = true;
          }
          config.HardwareLimitSwitch.ForwardLimitSource = T::forwardLimit_source;
        }
        if constexpr (has_forwardLimit_deviceID<T>{}) {
          static_assert(has_forwardLimit_source<T>{} &&
                            T::forwardLimit_source != ctre::phoenix6::signals::ForwardLimitSourceValue::Disabled &&
                            T::forwardLimit_source != ctre::phoenix6::signals::ForwardLimitSourceValue::LimitSwitchPin,
                        "Forward limit switch device ID has no effect when limit source is not remote");
          config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = T::forwardLimit_deviceID;
        }
        if constexpr (has_forwardLimit_normalState<T>{}) {
          if constexpr (T::forwardLimit_normalState != ctre::phoenix6::signals::ForwardLimitSourceValue::Disabled) {
            static_assert(has_forwardLimit_source<T>{}, "Forward limit switch source required");
          }
          config.HardwareLimitSwitch.ForwardLimitType = T::forwardLimit_normalState;
        }
      } else {
        config.HardwareLimitSwitch.ForwardLimitEnable = false;
      }
      if constexpr (has_reverseLimit_source<T>{} || has_reverseLimit_deviceID<T>{} ||
                    has_reverseLimit_normalState<T>{}) {
        if constexpr (has_reverseLimit_source<T>{}) {
          constexpr ctre::phoenix6::signals::ReverseLimitSourceValue source = T::reverseLimit_source;
          if constexpr (source != ctre::phoenix6::signals::ReverseLimitSourceValue::Disabled &&
                        source != ctre::phoenix6::signals::ReverseLimitSourceValue::LimitSwitchPin) {
            static_assert(has_reverseLimit_deviceID<T>{}, "Reverse limit switch requires remote source device ID");
          }
          if constexpr (source != ctre::phoenix6::signals::ReverseLimitSourceValue::Disabled) {
            static_assert(has_reverseLimit_normalState<T>{},
                          "Reverse limit switch configuration requires both source and normal state");
            config.HardwareLimitSwitch.ReverseLimitEnable = true;
          }
          config.HardwareLimitSwitch.ReverseLimitSource = T::reverseLimit_source;
        }
        if constexpr (has_reverseLimit_deviceID<T>{}) {
          static_assert(has_reverseLimit_source<T>{} &&
                            T::reverseLimit_source != ctre::phoenix6::signals::ReverseLimitSourceValue::Disabled &&
                            T::reverseLimit_source != ctre::phoenix6::signals::ReverseLimitSourceValue::LimitSwitchPin,
                        "Reverse limit switch device ID has no effect when limit source is not remote");
          config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = T::reverseLimit_deviceID;
        }
        if constexpr (has_reverseLimit_normalState<T>{}) {
          if constexpr (T::reverseLimit_normalState != ctre::phoenix6::signals::ReverseLimitSourceValue::Disabled) {
            static_assert(has_reverseLimit_source<T>{}, "Reverse limit switch source required");
          }
          config.HardwareLimitSwitch.ReverseLimitType = T::reverseLimit_normalState;
        }
      } else {
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
      }
      if constexpr (has_neutralDeadband<T>{}) {
        static_assert(T::neutralDeadband >= 0.001, "Neutral deadband must be greater than 0.001 (0.1%)");
        static_assert(T::neutralDeadband <= 0.25, "Neutral deadband must be less than 0.25 (25%)");
        config.MotorOutput.DutyCycleNeutralDeadband = T::neutralDeadband;
      }

      if constexpr (has_statusFrameMotorMode<T>()) {
        argos_lib::status_frame_config::SetMotorStatusFrameRates(motorController, T::statusFrameMotorMode);
      }

      auto retVal = motorController.GetConfigurator().Apply(config, configTimeout);
      if (0 != retVal) {
        std::cout << "Error code (" << motorController.GetDeviceID() << "): " << retVal << '\n';
      }

      return 0 != retVal;
    }

    /**
     * @brief Configures a CTRE Falcon with configuration values according to specified robot instance.
     *
     * @tparam CompetitionConfig Configurations to use in competition robot instance
     * @tparam PracticeConfig Configurations to use in practice robot instance
     * @param motorController Falcon object to configure
     * @param configTimeout Time to wait for response from Falcon
     * @param instance Robot instance to use
     * @return true Configuration succeeded
     * @return false Configuration failed
     */
    template <typename CompetitionConfig, typename PracticeConfig>
    bool FalconConfig(ctre::phoenix6::hardware::TalonFX& motorController,
                      units::millisecond_t configTimeout,
                      argos_lib::RobotInstance instance) {
      switch (instance) {
        case argos_lib::RobotInstance::Competition:
          return FalconConfig<CompetitionConfig>(motorController, configTimeout);
          break;
        case argos_lib::RobotInstance::Practice:
          return FalconConfig<PracticeConfig>(motorController, configTimeout);
          break;
      }
      return false;
    }

  }  // namespace falcon_config
}  // namespace argos_lib
