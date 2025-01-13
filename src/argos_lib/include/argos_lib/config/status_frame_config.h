/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/core/CoreTalonFX.hpp>

namespace argos_lib {
  namespace status_frame_config {
    using BaseTalon = ctre::phoenix6::hardware::core::CoreTalonFX;
    /**
     * @brief Motor mode used for setting status frame update periods
     */
    enum class MotorPresetMode {
      Basic,              ///< Baseline SRX, slow sensor reporting rate
      BasicFX,            ///< Baseline FX, slow sensor reporting rate
      Leader,             ///< SRX leader, fast reporting rate
      LeaderFX,           ///< FX leader, fast reporting rate
      Follower,           ///< SRX follower, near zero reporting rates
      FollowerFX,         ///< FX follower, near zero reporting rates
      MotionProfiling,    ///< SRX with motion profiling enabled
      MotionProfilingFX,  ///< FX with motion profiling enabled
      Tuning,             ///< SRX with increased reporting frequency to aid PID tuning
      TuningFX            ///< FX with increased reporting frequency to aid PID tuning
    };

    /**
     * @brief Set motor controller status frame update periods based on the motor preset
     *
     * @param motor Motor controller to configure
     * @param motorMode Preset mode to use
     */
    void SetMotorStatusFrameRates(BaseTalon& motor, MotorPresetMode motorMode);
  }  // namespace status_frame_config
}  // namespace argos_lib
