/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/general/odometry_aim.h"

#include <units/math.h>

#include <cmath>

units::degree_t argos_lib::odometry_aim::GetAngleToTarget(const frc::Translation2d& currentEstimatedRobotPose,
                                                          const frc::Translation3d& targetPoseOnField) {
  auto yawToTarget = units::math::atan2(targetPoseOnField.Y() - currentEstimatedRobotPose.Y(),
                                        targetPoseOnField.X() - currentEstimatedRobotPose.X());

  return 90.0_deg - yawToTarget;
}

units::meter_t argos_lib::odometry_aim::GetDistanceToTarget(const frc::Translation2d& currentEstimatedRobotPose,
                                                            const frc::Translation3d& targetPoseOnField) {
  auto Ydiff = targetPoseOnField.Y() - currentEstimatedRobotPose.Y();
  auto Xdiff = targetPoseOnField.X() - currentEstimatedRobotPose.X();
  return units::math::hypot(Ydiff, Xdiff);
}
