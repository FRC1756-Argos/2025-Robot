/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/time.h>

#include <functional>

namespace argos_lib {

  /**
   * @brief Vibration percentages that can be sent to controller for user feedback
   */
  struct VibrationStatus {
    double intensityLeft = 0.0;   ///< Intensity percent for left vibration [0,1.0]
    double intensityRight = 0.0;  ///< Intensity percent for right vibration [0,1.0]
  };

  using VibrationModel = std::function<VibrationStatus()>;

  /**
   * @brief Turn off vibration
   *
   * @return VibrationModel
   */
  VibrationModel VibrationOff();

  /**
   * @brief Set vibration to a constant value where left and right are the same intensity
   *
   * @param intensity Percent vibration output [0,1.0]
   * @return VibrationModel
   */
  VibrationModel VibrationConstant(double intensity);

  /**
   * @brief Set vibration to a constant value where left and right have discrete intensities
   *
   * @param intensityLeft Left vibration percent output [0,1.0]
   * @param intensityRight Right vibration percent output [0,1.0]
   * @return VibrationModel
   */
  VibrationModel VibrationConstant(double intensityLeft, double intensityRight);

  /**
   * @brief Toggle vibration between intensityOn and intensityOff levels.  Left and right
   *        have the same vibration intensity.
   *
   * @param pulsePeriod Time to complete one intensityOn->intensityOff cycle
   * @param intensityOn Vibration percent output when in intensityOn portion of cycle [0,1.0]
   * @param intensityOff Vibration percent output when in intensityOff portion of cycle [0,1.0]
   * @return VibrationModel
   */
  VibrationModel VibrationSyncPulse(units::millisecond_t pulsePeriod, double intensityOn, double intensityOff = 0.0);

  /**
   * @brief Smoothly transition vibration between intensityOn and intensityOff levels.  Left and right
   *        have the same vibration intensity.
   *
   * @param pulsePeriod Time to complete one intensityOn->intensityOff cycle
   * @param intensityOn Vibration percent output when at peak of intensityOn portion of cycle [0,1.0]
   * @param intensityOff Vibration percent output when in trough of intensityOff portion of cycle [0,1.0]
   * @return VibrationModel
   */
  VibrationModel VibrationSyncWave(units::millisecond_t pulsePeriod, double intensityOn, double intensityOff = 0.0);

  /**
   * @brief Toggle vibration between intensityOn and intensityOff levels.  Left output
   *        is in intensityOn phase while right output is in intensityOff phase and
   *        vice-versa.
   *
   * @param pulsePeriod Time to complete one intensityOn->intensityOff cycle
   * @param intensityOn Vibration percent output when in intensityOn portion of cycle [0,1.0]
   * @param intensityOff Vibration percent output when in intensityOff portion of cycle [0,1.0]
   * @return VibrationModel
   */
  VibrationModel VibrationAlternatePulse(units::millisecond_t pulsePeriod,
                                         double intensityOn,
                                         double intensityOff = 0.0);

  /**
   * @brief Smoothly transition vibration between intensityOn and intensityOff levels.  Left output
   *        is in intensityOn phase while right output is in intensityOff phase and
   *        vice-versa.
   *
   * @param pulsePeriod Time to complete one intensityOn->intensityOff cycle
   * @param intensityOn Vibration percent output when at peak of intensityOn portion of cycle [0,1.0]
   * @param intensityOff Vibration percent output when in trough of intensityOff portion of cycle [0,1.0]
   * @return VibrationModel
   */
  VibrationModel VibrationAlternateWave(units::millisecond_t pulsePeriod,
                                        double intensityOn,
                                        double intensityOff = 0.0);

  /**
   * @brief Run a vibration model for a specified duration, then run another model in perpetuity thereafter
   *
   * @param temporaryModel Model to run for the specified duration
   * @param temporaryModelDuration Amount of time to run temporaryModel
   * @param lastingModel Model to run forever after the specified duration
   * @return VibrationModel
   */
  VibrationModel TemporaryVibrationPattern(VibrationModel temporaryModel,
                                           units::millisecond_t temporaryModelDuration,
                                           VibrationModel lastingModel = VibrationOff());

}  // namespace argos_lib
