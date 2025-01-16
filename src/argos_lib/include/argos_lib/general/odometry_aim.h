/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>

#include "units/angle.h"
#include "units/length.h"

namespace argos_lib {
  namespace odometry_aim {
    /**
     * @brief Gets the Angle(Yaw) required for the Robot to orient towards the target of interest
     * assuming the robot is field-centric
     *
     * @param currentEstimatedRobotPose Fused pose of the robot currently on the field
     * @param targetPoseOnField Defines desired alias range
     * @return Angle(Yaw) of robot in degrees
     */
    units::degree_t GetAngleToTarget(const frc::Translation2d& currentEstimatedRobotPose,
                                     const frc::Translation3d& targetPoseOnField);

    /**
     * @brief Gets the Distance of the Robot to from the target of interest
     *
     * @param currentEstimatedRobotPose Fused pose of the robot currently on the field
     * @param targetPoseOnField Defines desired alias range
     * @return Distance in inches of robot from the target of interest
     */
    units::meter_t GetDistanceToTarget(const frc::Translation2d& currentEstimatedRobotPose,
                                       const frc::Translation3d& targetPoseOnField);
  }  // namespace odometry_aim
}  // namespace argos_lib
