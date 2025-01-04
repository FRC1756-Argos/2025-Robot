/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "constants/field_points.h"

frc::Pose2d utils::ReflectFieldPoint(const frc::Pose2d source) {
  units::degree_t targetAngle =
      units::math::abs(units::math::abs(source.Rotation().Degrees()) - 180_deg) <= 0.01_deg ?
          -source.Rotation().Degrees() :
          argos_lib::angle::NearestAngle(-source.Rotation().Degrees(), source.Rotation().Degrees());
  return frc::Pose2d{{source.X(), field_dimensions::fieldMaxY - source.Y()}, targetAngle};
}
