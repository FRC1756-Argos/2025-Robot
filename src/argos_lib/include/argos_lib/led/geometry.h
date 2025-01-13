/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/angle.h>

#include <optional>
#include <utility>

namespace argos_lib {
  namespace led {
    /**
     * @brief Flip an angle horizontally as though a ray bounces off a horizontal surface
     *
     * @param incidentAngle Angle of original motion
     * @return Reflected angle
     */
    units::degree_t FlipHorizontal(units::degree_t incidentAngle);

    /**
     * @brief Flip an angle vertically as though a ray bounces off a vertical surface
     *
     * @param incidentAngle Angle of original motion
     * @return Reflected angle
     */
    units::degree_t FlipVertical(units::degree_t inicidentAngle);

    /**
     * @brief Determine if a point (pointX, pointY) is within the bounds of an axis-aligned bounding
     *        box with opposite corners (corner1X, corner1Y) and (corner2X, corner2Y)
     *
     * @param corner1X First corner X coordinate
     * @param corner1Y First corner Y coordinate
     * @param corner2X Second corner X coordinate
     * @param corner2Y Second corner Y coordinate
     * @param pointX Test point X coordinate
     * @param pointY Test point Y coordinate
     * @return true if point is in bounding box
     */
    bool PointInBox(float corner1X, float corner1Y, float corner2X, float corner2Y, float pointX, float pointY);

    /**
     * @brief Finds the intersection point of two lines -- [(s1x1, s1y1), (s1x2, s1y2)] and
     *        [(s2x1, s2y1), (s2x2, s2y2)] -- if it exists.  If the two lines are colinear, no
     *        intersection point is returned.
     *
     * @param s1x1 First point of first segment x coordinate
     * @param s1y1 First point of first segment y coordinate
     * @param s1x2 Second point of first segment x coordinate
     * @param s1y2 Second point of first segment y coordinate
     * @param s2x1 First point of second segment x coordinate
     * @param s2y1 First point of second segment y coordinate
     * @param s2x2 Second point of second segment x coordinate
     * @param s2y2 Second point of second segment y coordinate
     * @return (x,y) pair indicating intersection point if it exists, std::nullopt otherwise
     */
    std::optional<std::pair<float, float>> SegmentIntersection(
        float s1x1, float s1y1, float s1x2, float s1y2, float s2x1, float s2y1, float s2x2, float s2y2);
  }  // namespace led
}  // namespace argos_lib
