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
    constexpr units::inch_t width{29.0};
    constexpr units::inch_t length{30.0};
  }  // namespace chassis
  namespace swerve_offsets {
    constexpr auto frontLeftLOffset = 3.25_in;
    constexpr auto frontLeftWOffset = 3.25_in;
    constexpr auto frontRightLOffset = 3.25_in;
    constexpr auto frontRightWOffset = 3.25_in;
    constexpr auto backRightWOffset = 3.25_in;
    constexpr auto backRightLOffset = 3.25_in;
    constexpr auto backLeftWOffset = 3.25_in;
    constexpr auto backLeftLOffset = 3.25_in;
  }  // namespace swerve_offsets
  namespace shooter_targets {
    constexpr auto speakerTagHeight = 58_in;  // needs update
    constexpr auto speakerOpeningHeightFromShooter = 54.5_in;
    constexpr auto trapOpeningHeight = 36.25_in;
    constexpr auto cameraOffsetFromShooter = 21.0_in;
    constexpr auto frontCamLateralOffsetFromShooter = 2.5_in;
    constexpr auto offsetDistanceThreshold = 200_in;
    constexpr auto offsetDistThresholdSecondaryCam = 240_in;
    constexpr auto offsetRotationThreshold = 50_deg;
    constexpr auto speakerOpeningHeightFromGround = 80.5_in;
    constexpr auto cameraHeightToAprilTag = 28.63_in;
    constexpr auto secondaryCameraToShooter = 27_in;
    constexpr auto passingShotBlueOffset = 15.0_deg;  // increase to go away from Amp
    constexpr auto passingShotRedOffset = 12.0_deg;   // decrease to go away from Amp
    constexpr double passingShotInertialFactor = 12.0;
    constexpr double frontSideSpinFactor = 0.0;
    constexpr double longShotSpinFactor = 0.0;
  }  // namespace shooter_targets
  namespace camera_front {
    constexpr auto cameraX = 0_in;  /// @todo real mounting offsets
    constexpr auto cameraZ = 7.25_in;
    constexpr auto cameraMountAngle = 24.9_deg;
    constexpr auto cameraHeight = 28.5_in;
    constexpr auto vFov = 24.85_deg * 2;
    constexpr auto hFov = 29.8_deg * 2;
  }                         // namespace camera_front
  namespace camera_back {}  // namespace camera_back

}  // namespace measure_up
