/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>

units::degree_t GetContinuousOffset(units::degree_t continuousStartRotation,
                                    frc::Rotation2d& startRotation,
                                    units::degree_t continuousEndRotation,
                                    frc::Rotation2d& endRotation);

units::degree_t GetContinuousOffset(units::degree_t continuousStartRotation,
                                    frc::Pose2d& startPosition,
                                    units::degree_t continuousEndRotation,
                                    frc::Pose2d& endPosition);
