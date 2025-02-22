/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <optional>

#include "interpolation.h"

namespace argos_lib {
  namespace swerve {

    /**
     * @brief Optimize swerve module to minimize rotations and drive direction changes
     *
     * @param desiredState Requested state.  Output must result in same motion
     * @param currentModuleAngle Module rotation angle in relative or absolute position
     * @param currentModuleAngularRate Current module rotation speed.  To prevent rapid changes in rotation direction.
     * @param currentModuleDriveVel Current module drive velocity.  To prevent rapid changes in drive motor velocity.
     * @param maxVelocity Max velocity for determining max transition change thresholds
     * @return Optimized swerve module state that results in same motion as desiredState
     */
    frc::SwerveModuleState Optimize(frc::SwerveModuleState desiredState,
                                    units::degree_t currentModuleAngle,
                                    units::degrees_per_second_t currentModuleAngularRate,
                                    units::feet_per_second_t currentModuleDriveVel,
                                    units::feet_per_second_t maxVelocity);

    /**
     * @brief Representation of the absolute encoder position of each module at home position
     */
    struct SwerveModulePositions {
      units::degree_t FrontLeft;
      units::degree_t FrontRight;
      units::degree_t RearRight;
      units::degree_t RearLeft;
    };

    /**
     * @brief Translation speeds as percent max output
     */
    struct TranslationSpeeds {
      double forwardSpeedPct;  ///< Speed with positive forward in range [-1, 1]
      double leftSpeedPct;     ///< Speed with positive left in range [-1, 1]
    };

    /**
     * @brief Use argos_lib::InterpolationMap to apply mapping according to joystick vector magnitude
     *
     * @tparam T Type of interpolated input
     * @param rawSpeeds Joystick inputs as percentages
     * @param interpMap Interpolation map to apply to magnitudes
     * @return Remapped joystick percentages.  Angle of vector will match rawSpeeds, but magnitude will
     *         scale according to interpMap.  This results in circular deadband and other mapping results.
     */
    template <class T>
    [[nodiscard]] constexpr TranslationSpeeds CircularInterpolate(const TranslationSpeeds rawSpeeds,
                                                                  const T interpMap) {
      const double magnitude = std::sqrt(std::pow(rawSpeeds.forwardSpeedPct, 2) + std::pow(rawSpeeds.leftSpeedPct, 2));
      const double angle = std::atan2(rawSpeeds.leftSpeedPct, rawSpeeds.forwardSpeedPct);
      const double mappedMagnitude = interpMap(magnitude);
      return TranslationSpeeds{mappedMagnitude * std::cos(angle), mappedMagnitude * std::sin(angle)};
    }

  }  // namespace swerve
}  // namespace argos_lib
