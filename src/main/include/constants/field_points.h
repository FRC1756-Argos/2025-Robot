/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/general/angle_utils.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/spline/Spline.h>
#include <units/angle.h>
#include <units/length.h>

#include <vector>

#include "measure_up.h"

namespace field_dimensions {
  // Field max x and y
  constexpr auto fieldMaxY = 315.5975_in;
  constexpr auto fieldMaxX = 651.2225_in;
  constexpr auto fieldMiddleX = fieldMaxX / 2;
}  // namespace field_dimensions

namespace utils {
  // * Note this will only work on points contained in friendly half of field
  /// @brief Reflects the point source over the middle of the field to get equivelent points accross the field
  /// @param source The point to reflect
  /// @return The reflected point accross the middle of the field
  constexpr frc::Translation3d ReflectFieldPoint(const frc::Translation3d source) {
    return frc::Translation3d{source.X(), field_dimensions::fieldMaxY - source.Y(), source.Z()};
  }

  constexpr frc::Translation2d ReflectFieldPoint(const frc::Translation2d source) {
    return frc::Translation2d{source.X(), field_dimensions::fieldMaxY - source.Y()};
  }

  constexpr units::angle::degree_t ReflectAngle(const units::angle::degree_t sourceAngle) {
    return sourceAngle * units::scalar_t{-1};
  }

  template <class T>
  std::vector<T> ReflectFieldPoint(const std::vector<T> source) {
    std::vector<T> retVal;
    retVal.reserve(source.size());
    std::transform(
        source.begin(), source.end(), std::back_inserter(retVal), [](T val) { return ReflectFieldPoint(val); });
    return retVal;
  }

  constexpr frc::Spline<3>::ControlVector ReflectFieldPoint(const frc::Spline<3>::ControlVector source) {
    return frc::Spline<3>::ControlVector{
        .x{source.x}, .y{units::meter_t{field_dimensions::fieldMaxY}.to<double>() - source.y[0], -source.y[1]}};
  }

  frc::Pose2d ReflectFieldPoint(const frc::Pose2d source);

  constexpr units::inch_t ReflectYLine(const units::inch_t source) {
    return field_dimensions::fieldMaxY - source;
  }
}  // namespace utils

namespace field_points {
  struct AprilTag {
    int id;
    frc::Translation3d pose;
    units::degree_t yaw;
    units::degree_t pitch;
  };

  namespace blue_alliance {
    // Reference game_piece_positions in Docs directory for conventions
    namespace game_pieces {}  // namespace game_pieces

    namespace april_tags_welded {
      constexpr AprilTag rh_pickup{12, {33.51_in, 25.80_in, 58.50_in}, 54_deg, 0_deg};
      constexpr AprilTag lh_pickup{13, {33.51_in, 291.20_in, 58.50_in}, 306_deg, 0_deg};
      constexpr AprilTag blue_barge{14, {325.68_in, 241.64_in, 73.54_in}, 180_deg, 30_deg};
      constexpr AprilTag red_barge{15, {325.68_in, 75.39_in, 73.54_in}, 180_deg, 30_deg};
      constexpr AprilTag processor{16, {235.73_in, 0.15_in, 51.25_in}, 90_deg, 0_deg};
      constexpr AprilTag reef_1{17, {160.39_in, 130.17_in, 12.13_in}, 240_deg, 0_deg};
      constexpr AprilTag reef_2{18, {144.00_in, 158.50_in, 12.13_in}, 180_deg, 0_deg};
      constexpr AprilTag reef_3{19, {160.39_in, 186.83_in, 12.13_in}, 120_deg, 0_deg};
      constexpr AprilTag reef_4{20, {193.10_in, 186.83_in, 12.13_in}, 60_deg, 0_deg};
      constexpr AprilTag reef_5{21, {209.49_in, 158.50_in, 12.13_in}, 0_deg, 0_deg};
      constexpr AprilTag reef_6{22, {193.10_in, 130.17_in, 12.13_in}, 300_deg, 0_deg};
    }  // namespace april_tags_welded

    namespace april_tags_andymark {
      constexpr AprilTag rh_pickup{12, {33.91_in, 24.73_in, 58.5_in}, 54_deg, 0_deg};
      constexpr AprilTag lh_pickup{13, {33.91_in, 291.9_in, 58.5_in}, 306_deg, 0_deg};
      constexpr AprilTag blue_barge{14, {325.68_in, 241.44_in, 73.54_in}, 180_deg, 30_deg};
      constexpr AprilTag red_barge{15, {325.68_in, 75.19_in, 73.54_in}, 180_deg, 30_deg};
      constexpr AprilTag processor{16, {238.49_in, 0.42_in, 51.25_in}, 90_deg, 0_deg};
      constexpr AprilTag reef_1{17, {160.39_in, 129.97_in, 12.13_in}, 240_deg, 0_deg};
      constexpr AprilTag reef_2{18, {144.0_in, 158.3_in, 12.13_in}, 180_deg, 0_deg};
      constexpr AprilTag reef_3{19, {160.39_in, 186.63_in, 12.13_in}, 120_deg, 0_deg};
      constexpr AprilTag reef_4{20, {193.1_in, 186.63_in, 12.13_in}, 60_deg, 0_deg};
      constexpr AprilTag reef_5{21, {209.49_in, 158.3_in, 12.13_in}, 0_deg, 0_deg};
      constexpr AprilTag reef_6{22, {193.1_in, 129.97_in, 12.13_in}, 300_deg, 0_deg};
    }  // namespace april_tags_andymark
  }  // namespace blue_alliance

  namespace red_alliance {
    namespace game_pieces {}  // namespace game_pieces

    namespace april_tags_welded {
      constexpr AprilTag lh_pickup{1, {657.37_in, 25.80_in, 58.50_in}, 126_deg, 0_deg};
      constexpr AprilTag rh_pickup{2, {657.37_in, 291.20_in, 58.50_in}, 234_deg, 0_deg};
      constexpr AprilTag processor{3, {455.15_in, 317.15_in, 51.25_in}, 270_deg, 0_deg};
      constexpr AprilTag blue_barge{4, {365.20_in, 241.64_in, 73.54_in}, 0_deg, 30_deg};
      constexpr AprilTag red_barge{5, {365.20_in, 75.39_in, 73.54_in}, 0_deg, 30_deg};
      constexpr AprilTag reef_1{6, {530.49_in, 130.17_in, 12.13_in}, 300_deg, 0_deg};
      constexpr AprilTag reef_2{7, {546.87_in, 158.50_in, 12.13_in}, 0_deg, 0_deg};
      constexpr AprilTag reef_3{8, {530.49_in, 186.83_in, 12.13_in}, 60_deg, 0_deg};
      constexpr AprilTag reef_4{9, {497.77_in, 186.83_in, 12.13_in}, 120_deg, 0_deg};
      constexpr AprilTag reef_5{10, {481.39_in, 158.50_in, 12.13_in}, 180_deg, 0_deg};
      constexpr AprilTag reef_6{11, {497.77_in, 130.17_in, 12.13_in}, 240_deg, 0_deg};
    }  // namespace april_tags_welded

    namespace april_tags_andymark {
      constexpr AprilTag lh_pickup{1, {656.98_in, 24.73_in, 58.5_in}, 126_deg, 0_deg};
      constexpr AprilTag rh_pickup{2, {656.98_in, 291.9_in, 58.5_in}, 234_deg, 0_deg};
      constexpr AprilTag processor{3, {452.4_in, 316.21_in, 51.25_in}, 270_deg, 0_deg};
      constexpr AprilTag blue_barge{4, {365.2_in, 241.44_in, 73.54_in}, 0_deg, 30_deg};
      constexpr AprilTag red_barge{5, {365.2_in, 75.19_in, 73.54_in}, 0_deg, 30_deg};
      constexpr AprilTag reef_1{6, {530.49_in, 129.97_in, 12.13_in}, 300_deg, 0_deg};
      constexpr AprilTag reef_2{7, {546.87_in, 158.3_in, 12.13_in}, 0_deg, 0_deg};
      constexpr AprilTag reef_3{8, {530.49_in, 186.63_in, 12.13_in}, 60_deg, 0_deg};
      constexpr AprilTag reef_4{9, {497.77_in, 186.63_in, 12.13_in}, 120_deg, 0_deg};
      constexpr AprilTag reef_5{10, {481.39_in, 158.3_in, 12.13_in}, 180_deg, 0_deg};
      constexpr AprilTag reef_6{11, {497.77_in, 129.97_in, 12.13_in}, 240_deg, 0_deg};
    }  // namespace april_tags_andymark
  }  // namespace red_alliance
}  // namespace field_points
