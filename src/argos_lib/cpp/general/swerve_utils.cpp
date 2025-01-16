/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/general/swerve_utils.h"

#include <cmath>

#include "argos_lib/general/angle_utils.h"

using argos_lib::angle::ConstrainAngle;
using argos_lib::angle::InvertedAngle;
using argos_lib::angle::NearestAngle;
using argos_lib::swerve::Optimize;

frc::SwerveModuleState argos_lib::swerve::Optimize(frc::SwerveModuleState desiredState,
                                                   units::degree_t currentModuleAngle,
                                                   units::degrees_per_second_t currentModuleAngularRate,
                                                   units::feet_per_second_t currentModuleDriveVel,
                                                   units::feet_per_second_t maxVelocity) {
  frc::SwerveModuleState closestForwardState{desiredState};
  frc::SwerveModuleState closestInverseState{desiredState};

  closestForwardState.angle = NearestAngle(desiredState.angle.Degrees(), currentModuleAngle);
  closestInverseState.angle = InvertedAngle(closestForwardState.angle.Degrees(), currentModuleAngle);
  closestInverseState.speed = closestInverseState.speed * -1.0;

  const auto fwdTurnSign = std::signbit((closestForwardState.angle.Degrees() - currentModuleAngle).to<double>());
  const auto revTurnSign = std::signbit((closestInverseState.angle.Degrees() - currentModuleAngle).to<double>());

  [[maybe_unused]] const auto velPreferFwd = currentModuleDriveVel > maxVelocity / 2;
  const auto velPreferRev = currentModuleDriveVel < -maxVelocity / 2;
  const auto angVelHasPreference = units::math::fabs(currentModuleAngularRate) > 20_deg_per_s;
  [[maybe_unused]] const auto angVelPreferFwd =
      angVelHasPreference && fwdTurnSign == std::signbit(currentModuleAngularRate.to<double>());
  const auto angVelPreferRev =
      angVelHasPreference && revTurnSign == std::signbit(currentModuleAngularRate.to<double>());

  const auto fwdDist = units::math::fabs(closestForwardState.angle.Degrees() - currentModuleAngle);
  const auto revDist = 180_deg - fwdDist;

  if (fwdDist < revDist && !(velPreferRev && angVelPreferRev)) {
    return closestForwardState;
  }
  return closestInverseState;
}
