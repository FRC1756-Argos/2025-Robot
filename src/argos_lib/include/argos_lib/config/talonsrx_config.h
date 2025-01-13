/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "argos_lib/config/config_types.h"
#include "compile_time_member_check.h"
#include "status_frame_config.h"

namespace argos_lib {
  namespace talonsrx_config {
    HAS_MEMBER(inverted)
    HAS_MEMBER(neutralMode)
    HAS_MEMBER(pid0_allowableError)
    HAS_MEMBER(pid0_iZone)
    HAS_MEMBER(pid0_kD)
    HAS_MEMBER(pid0_kF)
    HAS_MEMBER(pid0_kI)
    HAS_MEMBER(pid0_kP)
    HAS_MEMBER(pid0_selectedSensor)
    HAS_MEMBER(remoteFilter0_addr)
    HAS_MEMBER(remoteFilter0_type)
    HAS_MEMBER(sensorPhase)
    HAS_MEMBER(voltCompSat)
    HAS_MEMBER(statusFrameMotorMode)
    HAS_MEMBER(peakCurrentLimit)
    HAS_MEMBER(peakCurrentDuration)
    HAS_MEMBER(continuousCurrentLimit)
    HAS_MEMBER(peakOutputForward)
    HAS_MEMBER(peakOutputReverse)
    HAS_MEMBER(forwardLimitSwitchSource)
    HAS_MEMBER(reverseLimitSwitchSource)
    HAS_MEMBER(forwardLimitSwitchNormal)
    HAS_MEMBER(reverseLimitSwitchNormal)
    /**
     * @brief Configures a CTRE TalonSRX with only the fields provided.  All other fields
     *        are given the factory default values.
     *
     * @tparam T Structure containing any combination of the following members:
     *           - inverted
     *           - neutralMode
     *           - pid0_allowableError
     *           - pid0_iZone
     *           - pid0_kD,
     *           - pid0_kF
     *           - pid0_kI
     *           - pid0_kP
     *           - pid0_selectedSensor
     *           - remoteFilter0_addr,
     *           - remoteFilter0_type
     *           - sensorPhase
     *           - voltCompSat
     *           - statusFrameMotorMode
     *           - peakCurrentLimit
     *           - peakCurrentDuration
     *           - continuousCurrentLimit
     *           - peakOutputForward
     *           - peakOutputReverse
     *           - forwardLimitSwitchSource
     *           - reverseLimitSwitchSource
     *           - forwardLimitSwitchNormal
     *           - reverseLimitSwitchNormal
     * @param motorController TalonSRX object to configure
     * @param configTimeout Time to wait for response from TalonSRX
     * @return true Configuration succeeded
     * @return false Configuration failed
     */
    template <typename T>
    bool TalonSRXConfig(ctre::phoenix::motorcontrol::can::TalonSRX& motorController,
                        units::millisecond_t configTimeout) {
      ctre::phoenix::motorcontrol::can::TalonSRXConfiguration config;
      auto timeout = configTimeout.to<int>();

      if constexpr (has_inverted<T>{}) {
        motorController.SetInverted(T::inverted);
      }
      if constexpr (has_sensorPhase<T>{}) {
        motorController.SetSensorPhase(T::sensorPhase);
      }
      if constexpr (has_neutralMode<T>{}) {
        motorController.SetNeutralMode(T::neutralMode);
      }
      if constexpr (has_voltCompSat<T>{}) {
        constexpr units::volt_t voltage = T::voltCompSat;
        config.voltageCompSaturation = voltage.to<double>();
        motorController.EnableVoltageCompensation(true);
      } else {
        motorController.EnableVoltageCompensation(false);
      }
      if constexpr (has_remoteFilter0_addr<T>{} && has_remoteFilter0_type<T>{}) {
        ctre::phoenix::motorcontrol::can::FilterConfiguration filterConfig;
        filterConfig.remoteSensorDeviceID = T::remoteFilter0_addr.address;
        filterConfig.remoteSensorSource = T::remoteFilter0_type;
        config.remoteFilter0 = filterConfig;
      }
      if constexpr (has_pid0_selectedSensor<T>{}) {
        config.primaryPID.selectedFeedbackSensor = T::pid0_selectedSensor;
      }
      if constexpr (has_pid0_kP<T>{}) {
        config.slot0.kP = T::pid0_kP;
      }
      if constexpr (has_pid0_kI<T>{}) {
        config.slot0.kI = T::pid0_kI;
      }
      if constexpr (has_pid0_kD<T>{}) {
        config.slot0.kD = T::pid0_kD;
      }
      if constexpr (has_pid0_kF<T>{}) {
        config.slot0.kF = T::pid0_kF;
      }
      if constexpr (has_pid0_iZone<T>{}) {
        config.slot0.integralZone = T::pid0_iZone;
      }
      if constexpr (has_pid0_allowableError<T>{}) {
        config.slot0.allowableClosedloopError = T::pid0_allowableError;
      }
      if constexpr (has_peakCurrentLimit<T>()) {
        constexpr units::ampere_t currentLimit = T::peakCurrentLimit;
        static_assert(currentLimit.to<double>() > 0, "Current limit must be positive");
        config.peakCurrentLimit = std::round(currentLimit.to<double>());
      }
      if constexpr (has_peakCurrentDuration<T>()) {
        constexpr units::millisecond_t currentDuration = T::peakCurrentDuration;
        static_assert(currentDuration.to<double>() > 0, "Current duration must be positive");
        config.peakCurrentDuration = std::round(currentDuration.to<double>());
      }
      if constexpr (has_continuousCurrentLimit<T>()) {
        constexpr units::ampere_t currentLimit = T::continuousCurrentLimit;
        static_assert(currentLimit.to<double>() > 0, "Current limit must be positive");
        config.continuousCurrentLimit = std::round(currentLimit.to<double>());
      }
      if constexpr (has_peakOutputForward<T>()) {
        config.peakOutputForward = T::peakOutputForward;
      }
      if constexpr (has_peakOutputReverse<T>()) {
        config.peakOutputReverse = T::peakOutputReverse;
      }

      if constexpr (has_statusFrameMotorMode<T>()) {
        argos_lib::status_frame_config::SetMotorStatusFrameRates(motorController, T::statusFrameMotorMode);
      }

      if constexpr (has_forwardLimitSwitchSource<T>()) {
        config.forwardLimitSwitchSource = T::forwardLimitSwitchSource;
      }

      if constexpr (has_reverseLimitSwitchSource<T>()) {
        config.reverseLimitSwitchSource = T::reverseLimitSwitchSource;
      }

      if constexpr (has_forwardLimitSwitchNormal<T>()) {
        config.forwardLimitSwitchNormal = T::forwardLimitSwitchNormal;
      }

      if constexpr (has_reverseLimitSwitchNormal<T>()) {
        config.reverseLimitSwitchNormal = T::reverseLimitSwitchNormal;
      }

      // enable current limiting if any current limiting option is set, disable if none are
      if (has_continuousCurrentLimit<T>() || has_peakCurrentLimit<T>() || has_peakCurrentDuration<T>()) {
        motorController.EnableCurrentLimit(true);
      } else {
        motorController.EnableCurrentLimit(false);
      }

      // ensure using single threshold if peakCurrentLimit is not set but continuousCurrentLimit is
      if (!has_peakCurrentLimit<T>() && has_continuousCurrentLimit<T>()) {
        config.peakCurrentLimit = 0;
      }

      return 0 != motorController.ConfigAllSettings(config, timeout);
    }

    /**
     * @brief Configures a CTRE TalonSRX with configuration values according to specified robot instance.
     *
     * @tparam CompetitionConfig Configurations to use in competition robot instance
     * @tparam PracticeConfig Configurations to use in practice robot instance
     * @param motorController TalonSRX object to configure
     * @param configTimeout Time to wait for response from TalonSRX
     * @param instance Robot instance to use
     * @return true Configuration succeeded
     * @return false Configuration failed
     */
    template <typename CompetitionConfig, typename PracticeConfig>
    bool TalonSRXConfig(ctre::phoenix::motorcontrol::can::TalonSRX& motorController,
                        units::millisecond_t configTimeout,
                        argos_lib::RobotInstance instance) {
      switch (instance) {
        case argos_lib::RobotInstance::Competition:
          return TalonSRXConfig<CompetitionConfig>(motorController, configTimeout);
          break;
        case argos_lib::RobotInstance::Practice:
          return TalonSRXConfig<PracticeConfig>(motorController, configTimeout);
          break;
      }
      return false;
    }

  }  // namespace talonsrx_config
}  // namespace argos_lib
