/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/general/angle_utils.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <array>

namespace measure_up {
  constexpr auto bumperExtension = 3_in;  ///< Distance from frame to outer edge of bumpers
  namespace chassis {
    constexpr units::inch_t width{25.0};
    constexpr units::inch_t length{34.0};
  }  // namespace chassis
  namespace swerve_offsets {
    constexpr auto frontLeftLOffset = 2.625_in;
    constexpr auto frontLeftWOffset = 2.625_in;
    constexpr auto frontRightLOffset = 2.625_in;
    constexpr auto frontRightWOffset = 2.625_in;
    constexpr auto backRightWOffset = 2.625_in;
    constexpr auto backRightLOffset = 2.625_in;
    constexpr auto backLeftWOffset = 2.625_in;
    constexpr auto backLeftLOffset = 2.625_in;
  }  // namespace swerve_offsets

  namespace camera_front {
    constexpr auto cameraX = 0_in;  /// @todo real mounting offsets
    constexpr auto cameraZ = 7.25_in;
    constexpr auto cameraMountAngle = 24.9_deg;
    constexpr auto cameraHeight = 28.5_in;
    constexpr auto vFov = 24.85_deg * 2;
    constexpr auto hFov = 29.8_deg * 2;
  }  // namespace camera_front
  namespace camera_back {}  // namespace camera_back
  namespace elevator {
    namespace elevator {
      constexpr auto homeHeight = 10.1875_in;
      constexpr auto minHeight = 10.75_in;
      constexpr auto maxHeight = 65_in;
    }  // namespace elevator
    namespace arm {
      constexpr auto minAngle = -12_deg;
      constexpr auto maxAngle = 202_deg;
      constexpr auto internalMinAngle = 75_deg;
      constexpr auto internalMaxAngle = 105_deg;
    }  // namespace arm
    namespace wrist {
      constexpr auto minAngle = -90_deg;
      constexpr auto maxAngle = 90_deg;
    }  // namespace wrist
  }  // namespace elevator
  namespace climber {
    constexpr auto minAngle = -25_deg;
    constexpr auto maxAngle = 135_deg;
  }  // namespace climber
  namespace reef {
    constexpr auto leftReefScootDistance = 0.60_m + 0.5_in;
    constexpr auto rightReefScootDistance = 0.22_m + 2.5_in;
    constexpr auto reefToRobotCenterMinimum = 0.4_m + 1.25_in;
    constexpr auto reefToRobotCenterMinimumL1 = 0.4_m + 3.75_in;
    constexpr auto reefTagToCameraPlane = 0_deg;
    constexpr auto reefValidAlignmentDistance = 1.5_in;
    constexpr auto reefValidAlignmentAngle = 2_deg;
    constexpr auto reefErrorFloorForward = 0.5_in;
    constexpr auto reefErrorFloorLat = 0.5_in;
    constexpr auto visionMinSpeed = 0.045;
    constexpr auto visionMaxSpeed = 0.2;
  }  // namespace reef
}  // namespace measure_up
