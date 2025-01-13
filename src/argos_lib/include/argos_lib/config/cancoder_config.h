/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/time.h>

#include <ctre/phoenix6/CANcoder.hpp>

#include "compile_time_member_check.h"

namespace argos_lib {
  namespace cancoder_config {
    using CANcoder = ctre::phoenix6::hardware::CANcoder;

    HAS_MEMBER(direction)
    HAS_MEMBER(magOffset)
    HAS_MEMBER(absoluteSensorDiscontinuityPoint)

    /**
     * @brief Configures a CTRE CanCoder with only the fields provided.  All other fields
     *        are given the factory default values.
     *
     * @tparam T Structure containing any combination of the following members:
     *           - direction
     *           - magOffset
     *           - absoluteSensorDiscontinuityPoint
     * @param encoder CANcoder object to configure
     * @param configTimeout Time to wait for response from CANcoder
     * @return true Configuration succeeded
     * @return false Configuration failed
     */
    template <typename T>
    bool CanCoderConfig(CANcoder& encoder, units::millisecond_t configTimeout) {
      ctre::phoenix6::configs::CANcoderConfiguration config;
      encoder.GetConfigurator().Refresh(config.MagnetSensor, configTimeout);

      if constexpr (has_direction<T>{}) {
        config.MagnetSensor.SensorDirection = T::direction;
      }
      if constexpr (has_absoluteSensorDiscontinuityPoint<T>{}) {
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = T::absoluteSensorDiscontinuityPoint;
      }
      if constexpr (has_magOffset<T>{}) {
        config.MagnetSensor.WithMagnetOffset = T::magOffset;
      }

      return ctre::phoenix::StatusCode::OK == encoder.GetConfigurator().Apply(config, configTimeout);
    }

    /**
     * @brief Configures a CTRE CanCoder with configuration values according to specified robot instance.
     *
     * @tparam CompetitionConfig Configurations to use in competition robot instance
     * @tparam PracticeConfig Configurations to use in practice robot instance
     * @param encoder CANcoder object to configure
     * @param configTimeout Time to wait for response from CANcoder
     * @param instance Robot instance to use
     * @return true Configuration succeeded
     * @return false Configuration failed
     */
    template <typename CompetitionConfig, typename PracticeConfig>
    bool CanCoderConfig(CANcoder& encoder, units::millisecond_t configTimeout, argos_lib::RobotInstance instance) {
      switch (instance) {
        case argos_lib::RobotInstance::Competition:
          return CanCoderConfig<CompetitionConfig>(encoder, configTimeout);
          break;
        case argos_lib::RobotInstance::Practice:
          return CanCoderConfig<PracticeConfig>(encoder, configTimeout);
          break;
      }
      return false;
    }

  }  // namespace cancoder_config
}  // namespace argos_lib
