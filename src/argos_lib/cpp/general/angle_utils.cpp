/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/general/angle_utils.h"

#include <cmath>

#include "units/math.h"

using argos_lib::angle::ConstrainAngle;
using argos_lib::angle::InvertedAngle;
using argos_lib::angle::NearestAngle;

units::degree_t argos_lib::angle::NearestAngle(units::degree_t desiredAngle, units::degree_t referenceAngle) {
  const auto normalizedDesiredAngle = ConstrainAngle(desiredAngle, 0_deg, 360_deg);
  const auto normalizedReferenceAngle = ConstrainAngle(referenceAngle, 0_deg, 360_deg);

  auto angleDiff = normalizedDesiredAngle - normalizedReferenceAngle;

  // Closest equivalent angle is across discontinuity point
  if (units::math::fabs(angleDiff) > 180_deg) {
    angleDiff = units::math::copysign(360_deg - units::math::fabs(angleDiff), angleDiff * -1.0);
  }

  return referenceAngle + angleDiff;
}

units::degree_t argos_lib::angle::InvertedAngle(units::degree_t desiredAngle, units::degree_t referenceAngle) {
  // Inverted angle is 180 degrees offset from desired angle and in opposite travel direction from reference angle
  const auto fwDist = ConstrainAngle(desiredAngle - referenceAngle, -180_deg, 180_deg);
  const auto revDistMag = 180_deg - units::math::fabs(fwDist);
  return referenceAngle + units::math::copysign(revDistMag, -fwDist);
}

units::degree_t argos_lib::angle::ConstrainAngle(units::degree_t inVal,
                                                 units::degree_t minVal,
                                                 units::degree_t maxVal) {
  const auto range = maxVal - minVal;
  inVal = units::math::fmod(inVal - minVal, range);
  if (inVal < 0_deg) {
    inVal += range;
  }
  return inVal + minVal;
}

double argos_lib::angle::ConstrainAngle(double inVal, double minVal, double maxVal) {
  return ConstrainAngle(units::make_unit<units::degree_t>(inVal),
                        units::make_unit<units::degree_t>(minVal),
                        units::make_unit<units::degree_t>(maxVal))
      .to<double>();
}
