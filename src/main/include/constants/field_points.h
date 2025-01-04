/// \copyright Copyright (c) Argos FRC Team 1756.
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
  };

  namespace blue_alliance {
    // Reference game_piece_positions in Docs directory for conventions
    namespace game_pieces {}  // namespace game_pieces

    namespace april_tags {
      constexpr AprilTag amp{6, {72.5_in, 323.0_in, 53.38_in}, 0_deg};
      constexpr AprilTag speakerCenter{7, {-1.5_in, 218.42_in, 57.13_in}, 0_deg};
      constexpr AprilTag speakerInside{8, {-1.5_in, 196.17_in, 57.13_in}, 0_deg};
      constexpr AprilTag sourceRight{9, {14.02_in, 34.79_in, 53.38_in}, 60_deg};
      constexpr AprilTag sourceLeft{10, {57.54_in, 9.68_in, 53.38_in}, 60_deg};
      constexpr AprilTag stageLeft{15, {182.73_in, 177.10_in, 52_in}, 120_deg};
      constexpr AprilTag stageRight{16, {182.73_in, 146.19_in, 52_in}, 240_deg};
      constexpr AprilTag stageCenter{14, {209.48_in, 161.62_in, 52_in}, 0_deg};
    }  // namespace april_tags

  }  // namespace blue_alliance

  namespace red_alliance {
    namespace game_pieces {}  // namespace game_pieces

    /// @todo update tag translations and angles
    namespace april_tags {
      constexpr AprilTag amp{5, {}, 0_deg};
      constexpr AprilTag speakerCenter{4, {}, 0_deg};
      constexpr AprilTag speakerInside{3, {}, 0_deg};
      constexpr AprilTag sourceRight{1, {}, 0_deg};
      constexpr AprilTag sourceLeft{2, {}, 0_deg};
      constexpr AprilTag stageLeft{11, {}, 0_deg};
      constexpr AprilTag stageRight{12, {}, 0_deg};
      constexpr AprilTag stageCenter{13, {}, 0_deg};
    }  // namespace april_tags

  }  // namespace red_alliance
}  // namespace field_points
