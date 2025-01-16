/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include "units/angle.h"

namespace argos_lib {
  namespace angle {
    /**
     * @brief Finds closest angle alias of desiredAngle relative to referencedAngle.
     *        All inputs are normalized so ranges are unbounded.
     *
     * @param desiredAngle Angle to find alias for
     * @param referenceAngle Defines desired alias range
     * @return Angle alias of desiredAngle in range (referencedAngle - 180_deg, referencedAngle + 180_deg]
     */
    units::degree_t NearestAngle(units::degree_t desiredAngle, units::degree_t referenceAngle);

    /**
     * @brief Finds closest angle alias of a vector 180-degrees offset from desiredAngle relative to referencedAngle.
     *        All inputs are normalized so ranges are unbounded.
     *
     * @param desiredAngle Angle to find 180-degree offset alias for
     * @param referenceAngle Defines desired alias range
     * @return Offset angle alias of desiredAngle in range (referencedAngle - 180_deg, referencedAngle + 180_deg]
     */
    units::degree_t InvertedAngle(units::degree_t desiredAngle, units::degree_t referenceAngle);

    /**
     * @brief Normalize angle to specified range
     *
     * @param inVal Angle to constrain
     * @param minVal Normalization lower bound
     * @param maxVal Normalization upper bound
     * @return Normalized value in range [minVal, maxVal]
     */
    units::degree_t ConstrainAngle(units::degree_t inVal, units::degree_t minVal, units::degree_t maxVal);
    /**
     * @brief Normalize angle to specified range
     *
     * @param inVal Angle to constrain (degrees)
     * @param minVal Normalization lower bound (degrees)
     * @param maxVal Normalization upper bound (degrees)
     * @return Normalized value in range [minVal, maxVal] (degrees)
     */
    double ConstrainAngle(double inVal, double minVal, double maxVal);
  }  // namespace angle
}  // namespace argos_lib
