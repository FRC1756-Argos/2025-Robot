/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/led/geometry.h"

#include <algorithm>

units::degree_t argos_lib::led::FlipHorizontal(units::degree_t incidentAngle) {
  return 360_deg - incidentAngle;
}

units::degree_t argos_lib::led::FlipVertical(units::degree_t inicidentAngle) {
  return 180_deg - inicidentAngle;
}

bool argos_lib::led::PointInBox(
    float corner1X, float corner1Y, float corner2X, float corner2Y, float pointX, float pointY) {
  // Allow user to specify any two opposite corners of the axis-aligned bounding box and extract
  // the minimum and maximum x/y coordinates
  const auto [minX, maxX] = std::minmax(corner1X, corner2X);
  const auto [minY, maxY] = std::minmax(corner1Y, corner2Y);

  return pointX >= minX && pointX <= maxX && pointY >= minY && pointY <= maxY;
}

std::optional<std::pair<float, float>> argos_lib::led::SegmentIntersection(
    float s1x1, float s1y1, float s1x2, float s1y2, float s2x1, float s2y1, float s2x2, float s2y2) {
  // Get slope-intercept (y=mx+c) form of line colinear with each segment
  float slope1 = (s1y2 - s1y1) / (s1x2 - s1x1);
  float intercept1 = s1y1 - slope1 * s1x1;

  float slope2 = (s2y2 - s2y1) / (s2x2 - s2x1);
  float intercept2 = s2y1 - slope2 * s2x1;

  // If segments are colinear, there is no singular intersection point
  if (std::abs(slope1 - slope2) <= std::numeric_limits<float>::epsilon()) {
    return std::nullopt;
  }

  float intersectX = 0;
  float intersectY = 0;

  if (std::abs(s1x1 - s1x2) <= std::numeric_limits<float>::epsilon()) {
    // First segment is vertical
    intersectX = s1x1;
    intersectY = slope2 * intersectX + intercept2;
  } else if (std::abs(s2x1 - s2x2) <= std::numeric_limits<float>::epsilon()) {
    // Second segment is vertical
    intersectX = s2x1;
    intersectY = slope1 * intersectX + intercept1;
  } else {
    // Both segments have defined slope
    intersectX = (intercept2 - intercept1) / (slope1 - slope2);
    intersectY = slope1 * intersectX + intercept1;
  }

  // Once we know the lines intersect, ensure the intersection point is within the bounds of both segments
  if (PointInBox(s1x1, s1y1, s1x2, s1y2, intersectX, intersectY) &&
      PointInBox(s2x1, s2y1, s2x2, s2y2, intersectX, intersectY)) {
    return std::make_pair(intersectX, intersectY);
  }
  return std::nullopt;
}
