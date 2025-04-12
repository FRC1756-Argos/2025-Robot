/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <constants/feature_flags.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <vector>

#include <Constants.h>

#include "constants/field_points.h"
#include "ctre/phoenix6/Utils.hpp"
#include "limelight/LimelightHelpers.h"
#include "subsystems/vision_subsystem.h"

CameraInterface::CameraInterface() = default;

void CameraInterface::RequestTargetFilterReset() {
  m_target.ResetOnNextTarget();
}

VisionSubsystem::VisionSubsystem(const argos_lib::RobotInstance instance, SwerveDriveSubsystem* pDriveSubsystem)
    : m_instance(instance)
    , m_pDriveSubsystem(pDriveSubsystem)
    , m_usePolynomial(true)
    , m_useTrigonometry(false)
    , m_isAimWhileMoveActive(false)
    , m_enableStaticRotation(false)
    , m_isOdometryAimingActive(false)
    , m_isLeftAlignActive(false)
    , m_isRightAlignActive(false)
    , m_isAlgaeAlignActive(false)
    , m_isL1Active(false)
    , m_isAlgaeModeActive(false)
    , m_latestLeftHeartbeat(-1)
    , m_latestRightHeartbeat(-1)
    , m_leftFresh({0_ms, 50_ms}, false)
    , m_rightFresh({0_ms, 50_ms}, false)
    , m_latestReefSide(std::nullopt)
    , m_latestReefSpotTime()
    , m_leftCameraFrameUpdateSubscriber{leftCameraTableName}
    , m_rightCameraFrameUpdateSubscriber{rightCameraTableName}
    , m_yawUpdateThread{}
    , m_leftCameraMegaTag2PoseLogger{frc::DataLogManager::GetLog(), "leftCameraMegaTag2Pose"}
    , m_rightCameraMegaTag2PoseLogger{frc::DataLogManager::GetLog(), "rightCameraMegaTag2Pose"} {
  m_leftCameraFrameUpdateSubscriber.AddMonitor(
      "hb",
      [this](double) {
        LimelightHelpers::PoseEstimate mt2 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(leftCameraTableName);
        if (mt2.tagCount > 0 &&
            units::math::abs(m_pDriveSubsystem->GetIMUYawRate()) < units::degrees_per_second_t{360}) {
          /// @todo Get good odometry from vision
          // units::meter_t avgDist{mt2.avgTagDist};
          // const auto time =
          //     units::second_t{ctre::phoenix6::utils::GetCurrentTimeSeconds()} - units::millisecond_t{mt2.latency};
          // if (mt2.tagCount > 2 && avgDist < 15_ft) {
          //   m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {.2, .2, 9999999.0});
          // } else if (mt2.tagCount > 2 && avgDist < 25_ft) {
          //   m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {.6, .6, 9999999.0});
          // } else if (mt2.tagCount > 2 || avgDist < 15_ft) {
          //   m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {.9, .9, 9999999.0});
          // } else {
          //   m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {10.0, 10.0, 9999999.0});
          // }
          m_rightCameraMegaTag2PoseLogger.Append(mt2.pose, units::microsecond_t{mt2.timestampSeconds}.to<int64_t>());
        }
      },
      -1);
  m_rightCameraFrameUpdateSubscriber.AddMonitor(
      "hb",
      [this](double) {
        LimelightHelpers::PoseEstimate mt2 =
            LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(rightCameraTableName);
        if (mt2.tagCount > 0 &&
            units::math::abs(m_pDriveSubsystem->GetIMUYawRate()) < units::degrees_per_second_t{360}) {
          /// @todo Get good odometry from vision
          // units::meter_t avgDist{mt2.avgTagDist};
          // const auto time =
          //     units::second_t{ctre::phoenix6::utils::GetCurrentTimeSeconds()} - units::millisecond_t{mt2.latency};
          // if (mt2.tagCount > 2 && avgDist < 15_ft) {
          //   m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {.2, .2, 9999999.0});
          // } else if (mt2.tagCount > 2 && avgDist < 25_ft) {
          //   m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {.6, .6, 9999999.0});
          // } else if (mt2.tagCount > 2 || avgDist < 15_ft) {
          //   m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {.9, .9, 9999999.0});
          // } else {
          //   m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {10.0, 10.0, 9999999.0});
          // }
          m_leftCameraMegaTag2PoseLogger.Append(mt2.pose, units::microsecond_t{mt2.timestampSeconds}.to<int64_t>());
        }
      },
      -1);
  m_yawUpdateThread = std::jthread(std::bind_front(&VisionSubsystem::UpdateYaw, this));
  frc::DataLogManager::GetLog().AddStructSchema<frc::Pose2d>();
}

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
  std::shared_ptr<nt::NetworkTable> nt1 = nt::NetworkTableInstance::GetDefault().GetTable(leftCameraTableName);
  std::shared_ptr<nt::NetworkTable> nt2 = nt::NetworkTableInstance::GetDefault().GetTable(rightCameraTableName);

  auto latestLeftHeartbeat = nt1->GetNumber("hb", -1);
  auto latestRightHeartbeat = nt2->GetNumber("hb", -1);

  frc::SmartDashboard::PutBoolean("vision/Left fresh",
                                  m_leftFresh(latestLeftHeartbeat > 0 && latestLeftHeartbeat != m_latestLeftHeartbeat));
  frc::SmartDashboard::PutBoolean(
      "vision/Right fresh", m_rightFresh(latestRightHeartbeat > 0 && latestRightHeartbeat != m_latestRightHeartbeat));

  m_latestLeftHeartbeat = latestLeftHeartbeat;
  m_latestRightHeartbeat = latestRightHeartbeat;
}

bool VisionSubsystem::IsAimWhileMoveActive() {
  return m_isAimWhileMoveActive;
}

bool VisionSubsystem::IsOdometryAimingActive() {
  return m_isOdometryAimingActive;
}

void VisionSubsystem::SetOdometryAiming(bool val) {
  m_isOdometryAimingActive = val;
}

void VisionSubsystem::SetAimWhileMove(bool val) {
  m_isAimWhileMoveActive = val;
}

bool VisionSubsystem::IsStaticRotationEnabled() {
  return m_enableStaticRotation;
}

void VisionSubsystem::SetEnableStaticRotation(bool val) {
  m_enableStaticRotation = val;
}

void VisionSubsystem::SetPipeline(uint16_t tag) {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable(leftCameraTableName);

  if constexpr (feature_flags::nt_debugging) {
    frc::SmartDashboard::PutNumber("vision/Setting Pipeline", tag);
  }

  table->PutNumber("pipeline", tag);
}

void VisionSubsystem::RequestFilterReset() {
  m_cameraInterface.RequestTargetFilterReset();
}

LimelightTarget::tValues VisionSubsystem::GetLeftCameraTargetValues() {
  return m_cameraInterface.m_target.GetTarget(true, leftCameraTableName);
}

LimelightTarget::tValues VisionSubsystem::GetRightCameraTargetValues() {
  return m_cameraInterface.m_target.GetTarget(true, rightCameraTableName);
}

void VisionSubsystem::Disable() {
  SetPipeline(0);
  SetLeftAlign(false);
  SetRightAlign(false);
  SetAlgaeAlign(false);
  SetL1Active(false);
}

void VisionSubsystem::UpdateYaw(std::stop_token stopToken) {
  while (!stopToken.stop_requested()) {
    const auto latestPose = m_pDriveSubsystem->GetRawOdometry();
    LimelightHelpers::SetIMUMode(leftCameraTableName, 1);
    LimelightHelpers::SetRobotOrientation(
        leftCameraTableName, latestPose.Rotation().Degrees().to<double>(), 0, 0, 0, 0, 0);
    LimelightHelpers::SetIMUMode(rightCameraTableName, 1);
    LimelightHelpers::SetRobotOrientation(
        rightCameraTableName, latestPose.Rotation().Degrees().to<double>(), 0, 0, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds{20});
  }
}

std::optional<frc::Pose2d> VisionSubsystem::GetClosestReefTagPoseInCamSpace(bool resample) {
  const auto target = getWhichCamera(resample);
  if (target && target.value().cameraID == whichCamera::LEFT_CAMERA) {
    const auto leftTarget = target.value().target;
    frc::Rotation2d rotation{leftTarget.tagPoseCamSpace.Rotation().Y() - measure_up::reef::reefTagToCameraPlane};
    return frc::Pose2d{leftTarget.tagPoseCamSpace.Z() - measure_up::reef::reefToRobotCenterMinimum,
                       leftTarget.tagPoseCamSpace.X(),
                       rotation};
  } else if (target && target.value().cameraID == whichCamera::RIGHT_CAMERA) {
    const auto rightTarget = target.value().target;
    frc::Rotation2d rotation{rightTarget.tagPoseCamSpace.Rotation().Y() + measure_up::reef::reefTagToCameraPlane};
    return frc::Pose2d{rightTarget.tagPoseCamSpace.Z() - measure_up::reef::reefToRobotCenterMinimum,
                       rightTarget.tagPoseCamSpace.X(),
                       rotation};
  } else {
    return std::nullopt;
  }
}

std::optional<frc::Translation2d> VisionSubsystem::GetRobotSpaceReefAlignmentError(bool resample) {
  const auto target = getWhichCamera(resample);

  auto reefScootDistance = 0_m;
  auto reefToRobotMin = measure_up::reef::reefToRobotCenterMinimum;

  if (LeftAlignmentRequested()) {
    reefScootDistance = measure_up::reef::leftReefScootDistance;
    if (target && target.value().cameraID == whichCamera::LEFT_CAMERA) {
      reefScootDistance = measure_up::reef::rightReefScootDistance;
    }
  } else if (RightAlignmentRequested()) {
    reefScootDistance = measure_up::reef::rightReefScootDistance;
    if (target && target.value().cameraID == whichCamera::LEFT_CAMERA) {
      reefScootDistance = measure_up::reef::leftReefScootDistance;
    }
  }

  if (isL1Active()) {
    reefToRobotMin = measure_up::reef::reefToRobotCenterMinimumL1;
    if (reefScootDistance == measure_up::reef::leftReefScootDistance) {
      reefScootDistance += 1.5_in;
    } else {
      reefScootDistance -= 1.5_in;
    }
  }

  if (AlgaeAlignmentRequested() && isAlgaeModeActive()) {
    reefToRobotMin = measure_up::reef::reefToRobotCenterMinimumAlgae;
    reefScootDistance = measure_up::reef::algaeReefScootDistance;
  }

  if (target && target.value().cameraID == whichCamera::LEFT_CAMERA) {
    const auto leftTarget = target.value().target;
    frc::Translation2d robotCentricSpeeds(leftTarget.tagPoseRobotSpace.X() + reefToRobotMin,
                                          leftTarget.tagPoseRobotSpace.Z() + reefScootDistance);
    return robotCentricSpeeds;
  } else if (target && target.value().cameraID == whichCamera::RIGHT_CAMERA) {
    const auto rightTarget = target.value().target;
    frc::Translation2d robotCentricSpeeds(rightTarget.tagPoseRobotSpace.X() - reefToRobotMin,
                                          rightTarget.tagPoseRobotSpace.Z() + reefScootDistance);
    return robotCentricSpeeds;
  } else {
    return std::nullopt;
  }
}

std::optional<frc::Translation2d> VisionSubsystem::GetFieldCentricReefAlignmentError(bool resample) {
  const auto robotCentricSpeeds = GetRobotSpaceReefAlignmentError(resample);
  if (robotCentricSpeeds) {
    frc::Translation2d fieldCentricSpeeds =
        robotCentricSpeeds.value().RotateBy(m_pDriveSubsystem->GetFieldCentricAngle());

    frc::SmartDashboard::PutNumber("VisionSubsystem Transformed X (m/s)", fieldCentricSpeeds.X().to<double>());
    frc::SmartDashboard::PutNumber("VisionSubsystem Transformed Y (m/s)", fieldCentricSpeeds.Y().to<double>());

    return fieldCentricSpeeds;
  } else {
    return std::nullopt;
  }
}

std::optional<units::degree_t> VisionSubsystem::GetOrientationCorrection(bool resample) {
  const auto target = getWhichCamera(resample);
  if (target && target.value().cameraID == whichCamera::LEFT_CAMERA) {
    const auto leftTarget = target.value().target;
    frc::SmartDashboard::PutNumber("angle y", (leftTarget.tagPoseCamSpace.Rotation().Y()).value());
    return leftTarget.tagPoseCamSpace.Rotation().Y() - measure_up::reef::reefTagToCameraPlane;
  } else if (target && target.value().cameraID == whichCamera::RIGHT_CAMERA) {
    const auto rightTarget = target.value().target;
    frc::SmartDashboard::PutNumber("angle y", (rightTarget.tagPoseCamSpace.Rotation().Y()).value());
    return rightTarget.tagPoseCamSpace.Rotation().Y() + measure_up::reef::reefTagToCameraPlane;
  } else {
    return std::nullopt;
  }
}

void VisionSubsystem::SetLeftAlign(bool val) {
  if (val) {
    m_isRightAlignActive = false;
    m_isAlgaeAlignActive = false;
  }
  m_isRotationGoodEnough = false;
  m_isLeftAlignActive = val;
}

void VisionSubsystem::SetRightAlign(bool val) {
  if (val) {
    m_isLeftAlignActive = false;
    m_isAlgaeAlignActive = false;
  }
  m_isRotationGoodEnough = false;
  m_isRightAlignActive = val;
}

void VisionSubsystem::SetAlgaeAlign(bool val) {
  if (val) {
    m_isLeftAlignActive = false;
    m_isRightAlignActive = false;
  }
  m_isAlgaeAlignActive = val;
}

bool VisionSubsystem::LeftAlignmentRequested() {
  return m_isLeftAlignActive;
}

bool VisionSubsystem::RightAlignmentRequested() {
  return m_isRightAlignActive;
}

bool VisionSubsystem::AlgaeAlignmentRequested() {
  return m_isAlgaeAlignActive;
}

void VisionSubsystem::SetL1Active(bool val) {
  m_isL1Active = val;
}

bool VisionSubsystem::isL1Active() {
  return m_isL1Active;
}

void VisionSubsystem::SetAlgaeModeActive(bool val) {
  if (val) {
    m_isAlgaeModeActive = false;
  }
  m_isAlgaeModeActive = val;
}

bool VisionSubsystem::isAlgaeModeActive() {
  return m_isAlgaeModeActive;
}

std::optional<LimelightTarget::tValues> VisionSubsystem::GetSeeingCamera(bool resample) {
  const auto target = getWhichCamera(resample);
  if (target) {
    return target.value().target;
  } else {
    return std::nullopt;
  }
}

std::optional<ActiveVisionTarget> VisionSubsystem::getWhichCamera(bool resample) {
  static const std::vector<int> blueTagsOfInterest{field_points::blue_alliance::april_tags_welded::reef_1.id,
                                                   field_points::blue_alliance::april_tags_welded::reef_2.id,
                                                   field_points::blue_alliance::april_tags_welded::reef_3.id,
                                                   field_points::blue_alliance::april_tags_welded::reef_4.id,
                                                   field_points::blue_alliance::april_tags_welded::reef_5.id,
                                                   field_points::blue_alliance::april_tags_welded::reef_6.id};

  static const std::vector<int> redTagsOfInterest{field_points::red_alliance::april_tags_welded::reef_1.id,
                                                  field_points::red_alliance::april_tags_welded::reef_2.id,
                                                  field_points::red_alliance::april_tags_welded::reef_3.id,
                                                  field_points::red_alliance::april_tags_welded::reef_4.id,
                                                  field_points::red_alliance::april_tags_welded::reef_5.id,
                                                  field_points::red_alliance::april_tags_welded::reef_6.id};

  const auto& tagsOfInterest = (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) ?
                                   blueTagsOfInterest :
                                   redTagsOfInterest;

  if (!resample) {
    return (m_activeVisionTarget.cameraID != whichCamera::INVALID) ?
               std::optional<ActiveVisionTarget>{m_activeVisionTarget} :
               std::nullopt;
  }

  const auto leftCameraValues = GetLeftCameraTargetValues();
  const auto rightCameraValues = GetRightCameraTargetValues();

  if (std::find(tagsOfInterest.begin(), tagsOfInterest.end(), leftCameraValues.tagID) != tagsOfInterest.end() &&
      m_leftFresh.GetDebouncedStatus()) {
    m_latestReefSpotTime = std::chrono::steady_clock::now();
    m_latestReefSide = whichCamera::LEFT_CAMERA;

    m_activeVisionTarget.cameraID = whichCamera::LEFT_CAMERA;
    m_activeVisionTarget.sampleTime = std::chrono::steady_clock::now();
    m_activeVisionTarget.target = leftCameraValues;

    return m_activeVisionTarget;
  } else if (std::find(tagsOfInterest.begin(), tagsOfInterest.end(), rightCameraValues.tagID) != tagsOfInterest.end() &&
             m_rightFresh.GetDebouncedStatus()) {
    m_latestReefSpotTime = std::chrono::steady_clock::now();
    m_latestReefSide = whichCamera::RIGHT_CAMERA;

    m_activeVisionTarget.cameraID = whichCamera::RIGHT_CAMERA;
    m_activeVisionTarget.sampleTime = std::chrono::steady_clock::now();
    m_activeVisionTarget.target = rightCameraValues;

    return m_activeVisionTarget;
  } else {
    m_activeVisionTarget.cameraID = whichCamera::INVALID;
    return std::nullopt;
  }
}

std::optional<whichCamera> VisionSubsystem::getLatestReefSide() {
  // We haven't seen reef recently, discard latest reef side
  if ((std::chrono::steady_clock::now() - m_latestReefSpotTime) > std::chrono::seconds(15)) {
    m_latestReefSide = std::nullopt;
  }
  return m_latestReefSide;
}

// LIMELIGHT TARGET MEMBER FUNCTIONS ===============================================================

LimelightTarget::tValues LimelightTarget::GetTarget(bool filter, std::string cameraName) {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable(cameraName);

  auto rawRobotPose = table->GetNumberArray("botpose", std::span<const double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  m_robotPose = frc::Pose3d(frc::Translation3d(units::make_unit<units::meter_t>(rawRobotPose.at(0)),
                                               units::make_unit<units::meter_t>(rawRobotPose.at(1)),
                                               units::make_unit<units::meter_t>(rawRobotPose.at(2))),
                            frc::Rotation3d(units::make_unit<units::radian_t>(rawRobotPose.at(3)),
                                            units::make_unit<units::radian_t>(rawRobotPose.at(4)),
                                            units::make_unit<units::radian_t>(rawRobotPose.at(5))));
  auto rawRobotPoseWPI = table->GetNumberArray(
      frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ? "botpose_orb_wpiblue" :
                                                                                 "botpose_orb_wpired",
      std::span<const double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  m_robotPoseWPI = frc::Pose3d(frc::Translation3d(units::make_unit<units::meter_t>(rawRobotPoseWPI.at(0)),
                                                  units::make_unit<units::meter_t>(rawRobotPoseWPI.at(1)),
                                                  units::make_unit<units::meter_t>(rawRobotPoseWPI.at(2))),
                               frc::Rotation3d(units::make_unit<units::radian_t>(rawRobotPoseWPI.at(3)),
                                               units::make_unit<units::radian_t>(rawRobotPoseWPI.at(4)),
                                               units::make_unit<units::radian_t>(rawRobotPoseWPI.at(5))));
  auto tagPoseCamSpace =
      table->GetNumberArray("targetpose_cameraspace", std::span<const double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  m_targetPoseCamSpace = frc::Pose3d(frc::Translation3d(units::make_unit<units::meter_t>(tagPoseCamSpace.at(0)),
                                                        units::make_unit<units::meter_t>(tagPoseCamSpace.at(1)),
                                                        units::make_unit<units::meter_t>(tagPoseCamSpace.at(2))),
                                     frc::Rotation3d(units::make_unit<units::degree_t>(tagPoseCamSpace.at(3)),
                                                     units::make_unit<units::degree_t>(tagPoseCamSpace.at(4)),
                                                     units::make_unit<units::degree_t>(tagPoseCamSpace.at(5))));

  auto tagPoseRobotSpace =
      table->GetNumberArray("targetpose_robotspace", std::span<const double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  m_targetPoseRobotSpace = frc::Pose3d(frc::Translation3d(units::make_unit<units::meter_t>(tagPoseRobotSpace.at(0)),
                                                          units::make_unit<units::meter_t>(tagPoseRobotSpace.at(1)),
                                                          units::make_unit<units::meter_t>(tagPoseRobotSpace.at(2))),
                                       frc::Rotation3d(units::make_unit<units::degree_t>(tagPoseRobotSpace.at(3)),
                                                       units::make_unit<units::degree_t>(tagPoseRobotSpace.at(4)),
                                                       units::make_unit<units::degree_t>(tagPoseRobotSpace.at(5))));

  auto tagId = table->GetNumber("tid", 0.0);

  m_tid = tagId;
  m_hb = table->GetNumber("hb", -1);
  m_hasTargets = (table->GetNumber("tv", 0) == 1);
  m_yaw = units::make_unit<units::degree_t>(table->GetNumber("tx", 0.0));
  m_pitch = units::make_unit<units::degree_t>(table->GetNumber("ty", 0.0));

  if constexpr (feature_flags::nt_debugging) {
    frc::SmartDashboard::PutNumber("VisionSubsystem/RawPitch (deg)", m_pitch.to<double>());
    frc::SmartDashboard::PutNumber("VisionSubsystem/RawYaw (deg)", m_yaw.to<double>());
  }

  // If filter needs to reset, reset filter
  if (m_hasTargets && m_resetFilterFlag) {
    ResetFilters(cameraName);
  }

  // Filter incoming yaw & pitch if wanted
  if (filter && m_hasTargets) {
    m_yaw = m_txFilter.Calculate(m_yaw);
    m_pitch = m_tyFilter.Calculate(m_pitch);
    m_targetPoseCamSpace.Z() = m_zFilter.Calculate(m_targetPoseCamSpace.Z());

    if constexpr (feature_flags::nt_debugging) {
      frc::SmartDashboard::PutNumber("VisionSubsystem/FilteredPitch (deg)", m_pitch.to<double>());
      frc::SmartDashboard::PutNumber("VisionSubsystem/FilteredYaw (deg)", m_yaw.to<double>());
    }
  }

  m_area = (table->GetNumber("ta", 0.0));
  m_totalLatency = units::make_unit<units::millisecond_t>(rawRobotPose.at(6));

  return tValues{m_robotPose,
                 m_robotPoseWPI,
                 m_targetPoseCamSpace,
                 m_targetPoseRobotSpace,
                 m_hasTargets,
                 m_pitch,
                 m_yaw,
                 m_area,
                 m_tid,
                 m_totalLatency,
                 m_hb};
}

bool LimelightTarget::HasTarget() {
  return m_hasTargets;
}

void LimelightTarget::ResetOnNextTarget() {
  m_resetFilterFlag = true;
}

void LimelightTarget::ResetFilters(std::string cameraName) {
  m_resetFilterFlag = false;
  m_txFilter.Reset();
  m_tyFilter.Reset();
  m_zFilter.Reset();
  LimelightTarget::tValues currentValue = GetTarget(false, cameraName);
  // Hackily rest filter with initial value
  /// @todo name the filter values
  uint32_t samples = 0.7 / 0.02;
  for (size_t i = 0; i < samples; i++) {
    m_txFilter.Calculate(currentValue.m_yaw);
    m_tyFilter.Calculate(currentValue.m_pitch);
    m_zFilter.Calculate(currentValue.tagPoseCamSpace.Z());
  }
}

bool VisionSubsystem::robotAligned(bool resample) {
  auto alignmentError = GetRobotSpaceReefAlignmentError(resample);
  auto alignmentRotationError = GetOrientationCorrection(false);

  return alignmentError && alignmentRotationError &&
         (units::math::abs(alignmentError.value().Norm()) < measure_up::reef::reefValidAlignmentDistance) &&
         (units::math::abs(alignmentRotationError.value()) < 10.0_deg);
}

std::optional<unitlessChassisSpeeds> VisionSubsystem::getVisionAlignmentSpeeds(double scalingFactor) {
  unitlessChassisSpeeds speeds;

  if (LeftAlignmentRequested() || RightAlignmentRequested()) {
    auto robotToTagCorrections = GetRobotSpaceReefAlignmentError(true);
    auto robotRotationCorrection = GetOrientationCorrection(false);
    frc::SmartDashboard::PutNumber(
        "vision/AlignmentTimestamp",
        m_activeVisionTarget.cameraID == whichCamera::INVALID ?
            NAN :
            std::chrono::duration<double>(m_activeVisionTarget.sampleTime.time_since_epoch()).count());
    if (robotToTagCorrections && robotRotationCorrection) {
      units::meter_t forwardCorrection = robotToTagCorrections.value().X();
      units::degree_t rotationCorrection = robotRotationCorrection.value();
      units::meter_t lateralCorrection = robotToTagCorrections.value().Y();

      frc::SmartDashboard::PutNumber("vision/fwd correction (m)", forwardCorrection.value());
      frc::SmartDashboard::PutNumber("vision/rotation correction (deg)", rotationCorrection.value());
      frc::SmartDashboard::PutNumber("vision/lat correction (m)", lateralCorrection.value());

      speeds.ccwSpeed = -speeds::drive::rotationalProportionality * rotationCorrection.value();

      // Rotate slower when we are approaching the reef
      if (m_isRotationGoodEnough) {
        speeds.ccwSpeed *= 0.65;
      }

      // even though we have min and max speeds set, in general go lower using scaling factor
      // to give priority to smoothness and consistency during auto alignment
      double kP = scalingFactor * speeds::drive::translationalProportionality;

      if (lateralCorrection > 1_m || forwardCorrection > 1_m) {
        kP *= 0.5;
      } else {
        kP *= 0.65;
      }

      if (units::math::abs(rotationCorrection) < measure_up::reef::rotationThreshold) {
        m_isRotationGoodEnough = true;
      }

      // if robot managed to get out of range somehow, stop infinite translation/rotation and command the rotation only
      if (m_isRotationGoodEnough && units::math::abs(rotationCorrection) > measure_up::reef::rotationThreshold2) {
        m_isRotationGoodEnough = false;
      }

      // once we are almost oriented parallel to reef start zeroing down on the desired speeds
      if (m_isRotationGoodEnough) {
        if (units::math::abs(lateralCorrection) > measure_up::reef::reefErrorFloorForward) {
          speeds.forwardSpeed = kP * (lateralCorrection.value());
          if (std::abs(speeds.forwardSpeed) < measure_up::reef::visionMinSpeed) {
            speeds.forwardSpeed = (speeds.forwardSpeed < 0.0 ? -1.0 : 1.0) * measure_up::reef::visionMinSpeed;
          } else if (std::abs(speeds.forwardSpeed) > measure_up::reef::visionMaxSpeed) {
            speeds.forwardSpeed = (speeds.forwardSpeed < 0.0 ? -1.0 : 1.0) * measure_up::reef::visionMaxSpeed;
          }
        } else {
          speeds.forwardSpeed = 0;
        }
        // Approach towards reef after lateral to maintain consistency and predictability
        if (units::math::abs(forwardCorrection) > measure_up::reef::reefErrorFloorLat) {
          speeds.leftSpeed = -kP * (forwardCorrection.value());
          if (units::math::abs(lateralCorrection) > 10_in) {
            speeds.leftSpeed *= 0.65;
          }
          if (std::abs(speeds.leftSpeed) < measure_up::reef::visionMinSpeed) {
            speeds.leftSpeed = (speeds.leftSpeed < 0.0 ? -1.0 : 1.0) * measure_up::reef::visionMinSpeed;
          } else if (std::abs(speeds.leftSpeed) > measure_up::reef::visionMaxSpeed) {
            speeds.leftSpeed = (speeds.leftSpeed < 0.0 ? -1.0 : 1.0) * measure_up::reef::visionMaxSpeed;
          }
        } else {
          speeds.leftSpeed = 0;
        }
      }
    } else {
      frc::SmartDashboard::PutNumber("vision/fwd correction (m)", NAN);
      frc::SmartDashboard::PutNumber("vision/rotation correction (deg)", NAN);
      frc::SmartDashboard::PutNumber("vision/lat correction (m)", NAN);
      return std::nullopt;
    }
  } else {
    return std::nullopt;
  }

  frc::SmartDashboard::PutNumber("vision/speeds.leftSpeed", speeds.leftSpeed);
  frc::SmartDashboard::PutNumber("vision/speeds.forwardSpeed", speeds.forwardSpeed);
  frc::SmartDashboard::PutNumber("vision/speeds.ccwSpeed", speeds.ccwSpeed);
  return speeds;
}
