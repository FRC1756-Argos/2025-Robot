/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>

#include "measure_up.h"

struct Position {
  units::inch_t elevator_height;
  units::degree_t arm_angle;
  units::degree_t wrist_angle;

  [[nodiscard]] bool AlmostEqual(const Position other) {
    return units::math::abs(elevator_height - other.elevator_height) < 0.1_in &&
           units::math::abs(arm_angle - other.arm_angle) < 2_deg &&
           units::math::abs(wrist_angle - other.wrist_angle) < 2_deg;
  }
};

namespace setpoints {
  constexpr Position floorIntakeRight{
      .elevator_height = measure_up::elevator::elevator::minHeight, .arm_angle = 200_deg, .wrist_angle = 90_deg};
  constexpr Position floorIntakeLeft{
      .elevator_height = measure_up::elevator::elevator::minHeight, .arm_angle = -10_deg, .wrist_angle = -90_deg};
  constexpr Position coralStationRight{.elevator_height = 21_in, .arm_angle = 120_deg, .wrist_angle = 90_deg};
  constexpr Position coralStationLeft{.elevator_height = 21_in, .arm_angle = 60_deg, .wrist_angle = -90_deg};
  constexpr Position levelOneRight{
      .elevator_height = measure_up::elevator::elevator::minHeight, .arm_angle = 128_deg, .wrist_angle = 90_deg};
  constexpr Position levelOneLeft{
      .elevator_height = measure_up::elevator::elevator::minHeight, .arm_angle = 52_deg, .wrist_angle = -90_deg};
  constexpr Position levelTwoRight{.elevator_height = 21_in, .arm_angle = 134_deg, .wrist_angle = 0_deg};
  constexpr Position levelTwoLeft{.elevator_height = 21_in, .arm_angle = 49_deg, .wrist_angle = 0_deg};
  constexpr Position levelThreeRight{.elevator_height = 38_in, .arm_angle = 134_deg, .wrist_angle = 0_deg};
  constexpr Position levelThreeLeft{.elevator_height = 38_in, .arm_angle = 49_deg, .wrist_angle = 0_deg};
  constexpr Position levelFourRight{.elevator_height = 61.25_in, .arm_angle = 130_deg, .wrist_angle = 0_deg};
  constexpr Position levelFourLeft{.elevator_height = 61.25_in, .arm_angle = 58_deg, .wrist_angle = 0_deg};
  constexpr Position levelFourCenter{.elevator_height = 61.25_in, .arm_angle = 90_deg, .wrist_angle = 0_deg};
  constexpr Position stow{
      .elevator_height = measure_up::elevator::elevator::minHeight, .arm_angle = 90_deg, .wrist_angle = 0_deg};
}  // namespace setpoints

namespace algae {
  constexpr Position algaeLowLeft{.elevator_height = 33.5_in, .arm_angle = 0_deg, .wrist_angle = -90_deg};
  constexpr Position algaeLowRight{.elevator_height = 33.5_in, .arm_angle = 180_deg, .wrist_angle = 90_deg};
  constexpr Position algaeHighLeft{.elevator_height = 49.5_in, .arm_angle = 0_deg, .wrist_angle = -90_deg};
  constexpr Position algaeHighRight{.elevator_height = 49.5_in, .arm_angle = 180_deg, .wrist_angle = 90_deg};
  constexpr Position algaeNetLeft{
      .elevator_height = measure_up::elevator::elevator::maxHeight, .arm_angle = 58_deg, .wrist_angle = -90_deg};
  constexpr Position algaeNetRight{
      .elevator_height = measure_up::elevator::elevator::maxHeight, .arm_angle = 122_deg, .wrist_angle = 90_deg};
  constexpr Position algaeProcessorLeft{
      .elevator_height = measure_up::elevator::elevator::minHeight, .arm_angle = 0_deg, .wrist_angle = -90_deg};
  constexpr Position algaeProcessorRight{
      .elevator_height = measure_up::elevator::elevator::minHeight, .arm_angle = 180_deg, .wrist_angle = 90_deg};
}  // namespace algae

namespace internal {
  constexpr Position lowLeft{.elevator_height = 0_in, .arm_angle = 5_deg, .wrist_angle = -90_deg};
  constexpr Position lowRight{.elevator_height = 0_in, .arm_angle = 175_deg, .wrist_angle = 90_deg};
  constexpr Position highLeft{.elevator_height = 0_in, .arm_angle = 70_deg, .wrist_angle = 0_deg};
  constexpr Position highRight{.elevator_height = 0_in, .arm_angle = 110_deg, .wrist_angle = 0_deg};
}  // namespace internal
