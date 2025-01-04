/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/pose_continuity_fix.h"

units::degree_t GetContinuousOffset(units::degree_t continuousStartRotation,
                                    frc::Rotation2d& startRotation,
                                    units::degree_t continuousEndRotation,
                                    frc::Rotation2d& endRotation) {
  units::degree_t angleDifference = continuousEndRotation - continuousStartRotation;

  if (startRotation.Degrees() + angleDifference > 180_deg) {
    units::degree_t continuousOffset = (180_deg - startRotation.Degrees()) - angleDifference;
    startRotation = frc::Rotation2d{startRotation.Degrees() + continuousOffset};
    endRotation = frc::Rotation2d{startRotation.Degrees() + angleDifference};
    return continuousOffset;
  } else if (endRotation.Degrees() + angleDifference < -180_deg) {
    units::degree_t continuousOffset = (-180_deg - startRotation.Degrees()) - angleDifference;
    startRotation = frc::Rotation2d{startRotation.Degrees() + continuousOffset};
    endRotation = frc::Rotation2d{startRotation.Degrees() + angleDifference};
    return continuousOffset;
  }
  return 0_deg;
}

units::degree_t GetContinuousOffset(units::degree_t continuousStartRotation,
                                    frc::Pose2d& startPosition,
                                    units::degree_t continuousEndRotation,
                                    frc::Pose2d& endPosition) {
  frc::Rotation2d startRotation(startPosition.Rotation());
  frc::Rotation2d endRotation(endPosition.Rotation());

  units::degree_t offset =
      GetContinuousOffset(continuousStartRotation, startRotation, continuousEndRotation, endRotation);

  startPosition = frc::Pose2d{startPosition.Translation(), startRotation};
  endPosition = frc::Pose2d{endPosition.Translation(), endRotation};

  return offset;
}
