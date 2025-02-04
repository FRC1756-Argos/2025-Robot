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
      constexpr auto maxHeight = 62_in;
      constexpr auto floorIntakeHeightRight = minHeight;
      constexpr auto floorIntakeHeightLeft = minHeight;
      constexpr auto coralStationHeightRight = 21_in;
      constexpr auto coralStationHeightLeft = 21_in;
      constexpr auto levelOneHeightRight = minHeight;
      constexpr auto levelOneHeightLeft = minHeight;
      constexpr auto levelTwoHeightRight = 17_in;
      constexpr auto levelTwoHeightLeft = 17_in;
      constexpr auto levelThreeHeightRight = 33_in;
      constexpr auto levelThreeHeightLeft = 33_in;
      constexpr auto levelFourHeightRIght = 65_in;
      constexpr auto levelFourHeightLeft = 65_in;
      constexpr auto stowHeight = minHeight;
    }  // namespace elevator
    namespace arm {
      constexpr auto minAngle = -10_deg;
      constexpr auto maxAngle = 190_deg;
      constexpr auto floorIntakeAngleRight = 190_deg;
      constexpr auto floorIntakeAngleLeft = -10_deg;
      constexpr auto coralStationAngleRight = 124_deg;  //this is still needed//
      constexpr auto coralStationAngleLeft = 56_deg;    //this is still needed//
      constexpr auto levelOneAngleRight = 128_deg;
      constexpr auto levelOneAngleLeft = 52_deg;
      constexpr auto levelTwoAngleRight = 116_deg;
      constexpr auto levelTwoAngleLeft = 64_deg;
      constexpr auto levelThreeAngleRight = 116_deg;
      constexpr auto levelThreeAngleLeft = 64_deg;
      constexpr auto levelFourAngleRIght = 112_deg;
      constexpr auto levelFourAngleLeft = 68_deg;
      constexpr auto stowAngle = 90_deg;
    }  // namespace arm
    namespace wrist {
      constexpr auto minAngle = -90_deg;
      constexpr auto maxAngle = 90_deg;
      constexpr auto floorIntakeAngleRight = 90_deg;
      constexpr auto floorIntakeAngleLeft = -90_deg;
      constexpr auto coralStationAngleRight = 90_deg;
      constexpr auto coralStationAngleLeft = -90_deg;
      constexpr auto levelOneAngleRight = 90_deg;
      constexpr auto levelOneAngleLeft = -90_deg;
      constexpr auto levelTwoAngleRight = 0_deg;
      constexpr auto levelTwoAngleLeft = 0_deg;
      constexpr auto levelThreeAngleRight = 0_deg;
      constexpr auto levelThreeAngleLeft = 0_deg;
      constexpr auto levelFourAngleRIght = 0_deg;
      constexpr auto levelFourAngleLeft = 0_deg;
      constexpr auto stowAngle = 0_deg;
    }  // namespace wrist
  }  // namespace elevator
}  // namespace measure_up
