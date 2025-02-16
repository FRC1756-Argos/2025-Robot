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
    , m_primaryCameraFrameUpdateSubscriber{primaryCameraTableName}
    , m_secondaryCameraFrameUpdateSubscriber{secondaryCameraTableName}
    , m_yawUpdateThread{}
    , m_frontCameraMegaTag2PoseLogger{frc::DataLogManager::GetLog(), "frontCameraMegaTag2Pose"}
    , m_rearCameraMegaTag2PoseLogger{frc::DataLogManager::GetLog(), "rearCameraMegaTag2Pose"} {
  m_primaryCameraFrameUpdateSubscriber.AddMonitor(
      "hb",
      [this](double) {
        LimelightHelpers::PoseEstimate mt2 =
            LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(primaryCameraTableName);
        if (mt2.tagCount > 0 &&
            units::math::abs(m_pDriveSubsystem->GetIMUYawRate()) < units::degrees_per_second_t{360}) {
          units::meter_t avgDist{mt2.avgTagDist};
          const auto time =
              units::second_t{ctre::phoenix6::utils::GetCurrentTimeSeconds()} - units::millisecond_t{mt2.latency};
          if (mt2.tagCount > 2 && avgDist < 15_ft) {
            m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {.2, .2, 9999999.0});
          } else if (mt2.tagCount > 2 && avgDist < 25_ft) {
            m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {.6, .6, 9999999.0});
          } else if (mt2.tagCount > 2 || avgDist < 15_ft) {
            m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {.9, .9, 9999999.0});
          } else {
            m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {10.0, 10.0, 9999999.0});
          }
          m_rearCameraMegaTag2PoseLogger.Append(mt2.pose, units::microsecond_t{mt2.timestampSeconds}.to<int64_t>());
        }
      },
      -1);
  m_secondaryCameraFrameUpdateSubscriber.AddMonitor(
      "hb",
      [this](double) {
        LimelightHelpers::PoseEstimate mt2 =
            LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(secondaryCameraTableName);
        if (mt2.tagCount > 0 &&
            units::math::abs(m_pDriveSubsystem->GetIMUYawRate()) < units::degrees_per_second_t{360}) {
          units::meter_t avgDist{mt2.avgTagDist};
          const auto time =
              units::second_t{ctre::phoenix6::utils::GetCurrentTimeSeconds()} - units::millisecond_t{mt2.latency};
          if (mt2.tagCount > 2 && avgDist < 15_ft) {
            m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {.2, .2, 9999999.0});
          } else if (mt2.tagCount > 2 && avgDist < 25_ft) {
            m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {.6, .6, 9999999.0});
          } else if (mt2.tagCount > 2 || avgDist < 15_ft) {
            m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {.9, .9, 9999999.0});
          } else {
            m_pDriveSubsystem->UpdateVisionMeasurement(mt2.pose, time, {10.0, 10.0, 9999999.0});
          }
          m_frontCameraMegaTag2PoseLogger.Append(mt2.pose, units::microsecond_t{mt2.timestampSeconds}.to<int64_t>());
        }
      },
      -1);
  m_yawUpdateThread = std::jthread(std::bind_front(&VisionSubsystem::UpdateYaw, this));
  frc::DataLogManager::GetLog().AddStructSchema<frc::Pose2d>();
}

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
  std::shared_ptr<nt::NetworkTable> nt1 = nt::NetworkTableInstance::GetDefault().GetTable(primaryCameraTableName);
  std::shared_ptr<nt::NetworkTable> nt2 = nt::NetworkTableInstance::GetDefault().GetTable(secondaryCameraTableName);
  // int tagOfInterest = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                         field_points::blue_alliance::april_tags::speakerCenter.id :
  //                         field_points::red_alliance::april_tags::speakerCenter.id;
  // nt1->PutNumber("priorityid", tagOfInterest);
  // nt2->PutNumber("priorityid", tagOfInterest);

  // if constexpr (feature_flags::nt_debugging) {
  //   const auto targetValues = GetSeeingCamera();  // Note that this will update the targets object

  //   if (targetValues && targetValues.value().hasTargets) {
  //     frc::SmartDashboard::PutBoolean("(Vision - Periodic) Is Target Present?", targetValues.value().hasTargets);
  //     frc::SmartDashboard::PutNumber("(Vision - Periodic) Target Pitch", targetValues.value().m_pitch.to<double>());
  //     frc::SmartDashboard::PutNumber("(Vision - Periodic) Target Yaw", targetValues.value().m_yaw.to<double>());
  //     frc::SmartDashboard::PutNumber("(Vision - Periodic) Tag ID", targetValues.value().tagID);

  //     auto dist = GetDistanceToSpeaker();
  //     if (dist != std::nullopt) {
  //       frc::SmartDashboard::PutNumber("(Vision - Periodic) Tag Distance from Camera", dist.value().to<double>());
  //     }
  //   }

  //   auto calcDist = GetCalculatedDistanceToSpeaker();
  //   if (calcDist = std::nullopt) {
  //     frc::SmartDashboard::PutNumber("(Vision - Periodic) Calculated Tag Distance from Camera",
  //                                    calcDist.value().to<double>());
  //   }
  // }
}

std::optional<units::degree_t> VisionSubsystem::GetHorizontalOffsetToTarget() {
  if (!IsOdometryAimingActive()) {
    // Updates and retrieves new target values
    // const auto targetValues = GetSeeingCamera();
    // int tagOfInterest = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
    //                         field_points::blue_alliance::april_tags::speakerCenter.id :
    //                         field_points::red_alliance::april_tags::speakerCenter.id;
    // if (targetValues && tagOfInterest == targetValues.value().tagID) {
    //   // add more target validation after testing e.g. area, margin, skew etc
    //   // for now has target is enough as we will be fairly close to target
    //   // and will tune the pipeline not to combine detections and choose the highest area
    //   if (targetValues.value().hasTargets) {
    //     return targetValues.value().m_yaw;
    //   }
    // }
  } else {  // Odometry aiming
    // auto currentRobotAngle = m_pDriveSubsystem->GetFieldCentricAngle();
    // const auto targetPose = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
    //                             field_points::blue_alliance::april_tags::speakerCenter.pose :
    //                             field_points::red_alliance::april_tags::speakerCenter.pose;
    // return -(currentRobotAngle - argos_lib::odometry_aim::GetAngleToTarget(
    //                                  m_pDriveSubsystem->GetContinuousOdometry().Translation(), targetPose));
  }

  return std::nullopt;
}

std::optional<units::degree_t> VisionSubsystem::getFeederOffset() {
  // const auto targetValues = GetSeeingCamera(true);
  // int tagOfInterest = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                         field_points::blue_alliance::april_tags::stageCenter.id :
  //                         field_points::red_alliance::april_tags::stageCenter.id;

  // if (targetValues && tagOfInterest == targetValues.value().tagID) {
  //   if (targetValues.value().hasTargets) {
  //     return targetValues.value().m_yaw;
  //   }
  // }

  return std::nullopt;
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

std::optional<double> VisionSubsystem::getRotationSpeedWithInertia(double lateralSpeedPct) {
  // auto horzOffset = GetHorizontalOffsetToTarget();
  // auto shooterOffset = getShooterOffset();

  // if (horzOffset && shooterOffset) {
  //   double offset = horzOffset.value().to<double>();
  //   offset -= shooterOffset.value().to<double>();
  //   double finalOffset = offset - speeds::drive::lateralInertialWeight * lateralSpeedPct;

  //   return (-finalOffset * speeds::drive::rotationalProportionality);
  // }

  return std::nullopt;
}

std::optional<units::inch_t> VisionSubsystem::GetDistanceToSpeaker() {
  // int tagOfInterest = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                         field_points::blue_alliance::april_tags::speakerCenter.id :
  //                         field_points::red_alliance::april_tags::speakerCenter.id;

  // const auto targetValues = GetSeeingCamera();
  // const auto camera = getWhichCamera();
  // if (targetValues && tagOfInterest == targetValues.value().tagID) {
  //   if (camera && camera.value() == whichCamera::SECONDARY_CAMERA)
  //     return (static_cast<units::inch_t>(targetValues.value().tagPose.Z()) + 0_in);
  //   else
  //     return static_cast<units::inch_t>(targetValues.value().tagPose.Z());
  // } else {
  return std::nullopt;
  // }
}

std::optional<units::inch_t> VisionSubsystem::GetDistanceToStageCenter() {
  // int tagOfInterest = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                         field_points::blue_alliance::april_tags::stageCenter.id :
  //                         field_points::red_alliance::april_tags::stageCenter.id;

  // const auto targetValues = GetSeeingCamera(true);
  // if (targetValues && tagOfInterest == targetValues.value().tagID) {
  //   return static_cast<units::inch_t>(targetValues.value().tagPose.Z());
  // } else {
  return std::nullopt;
  // }
}

std::optional<units::degree_t> VisionSubsystem::GetOrientationToSpeaker() {
  // int tagOfInterest = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                         field_points::blue_alliance::april_tags::speakerCenter.id :
  //                         field_points::red_alliance::april_tags::speakerCenter.id;
  // const auto targetValues = GetSeeingCamera();
  // if (targetValues && tagOfInterest == targetValues.value().tagID)
  //   return static_cast<units::degree_t>(targetValues.value().tagPose.Rotation().Z());
  // else
  return std::nullopt;
}

std::optional<units::inch_t> VisionSubsystem::GetCalculatedDistanceToSpeaker() {
  // const auto targetValues = GetSeeingCamera();
  // int tagOfInterest = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                         field_points::blue_alliance::april_tags::speakerCenter.id :
  //                         field_points::red_alliance::april_tags::speakerCenter.id;
  // if (targetValues && tagOfInterest == targetValues.value().tagID) {
  //   return (0_in - measure_up::camera_front::cameraHeight) /
  //          std::tan(
  //              static_cast<units::radian_t>(measure_up::camera_front::cameraMountAngle + targetValues.value().m_pitch)
  //                  .to<double>());
  // } else {
  return std::nullopt;
  // }
}

std::optional<units::inch_t> VisionSubsystem::GetDistanceToTrap() {
  // int tagOfInterest1 = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                          field_points::blue_alliance::april_tags::stageCenter.id :
  //                          field_points::red_alliance::april_tags::stageCenter.id;
  // int tagOfInterest2 = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                          field_points::blue_alliance::april_tags::stageLeft.id :
  //                          field_points::red_alliance::april_tags::stageLeft.id;
  // int tagOfInterest3 = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                          field_points::blue_alliance::april_tags::stageRight.id :
  //                          field_points::red_alliance::april_tags::stageRight.id;
  // const auto targetValues = GetSeeingCamera();
  // if (targetValues && (tagOfInterest1 == targetValues.value().tagID || tagOfInterest2 == targetValues.value().tagID ||
  //                      tagOfInterest3 == targetValues.value().tagID)) {
  //   return static_cast<units::inch_t>(targetValues.value().tagPose.Z());
  // } else {
  return std::nullopt;
  // }
}

std::optional<units::degree_t> VisionSubsystem::GetHorizontalOffsetToTrap() {
  // const auto targetValues = GetSeeingCamera();
  // int tagOfInterest1 = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                          field_points::blue_alliance::april_tags::stageCenter.id :
  //                          field_points::red_alliance::april_tags::stageCenter.id;
  // int tagOfInterest2 = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                          field_points::blue_alliance::april_tags::stageLeft.id :
  //                          field_points::red_alliance::april_tags::stageLeft.id;
  // int tagOfInterest3 = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                          field_points::blue_alliance::april_tags::stageRight.id :
  //                          field_points::red_alliance::april_tags::stageRight.id;
  // if (targetValues && (tagOfInterest1 == targetValues.value().tagID || tagOfInterest2 == targetValues.value().tagID ||
  //                      tagOfInterest3 == targetValues.value().tagID)) {
  //   if (targetValues.value().hasTargets) {
  //     return targetValues.value().m_yaw;
  //   }
  // }

  return std::nullopt;
}

std::optional<units::degree_t> VisionSubsystem::GetOrientationToTrap() {
  // int tagOfInterest1 = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                          field_points::blue_alliance::april_tags::stageCenter.id :
  //                          field_points::red_alliance::april_tags::stageCenter.id;
  // int tagOfInterest2 = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                          field_points::blue_alliance::april_tags::stageLeft.id :
  //                          field_points::red_alliance::april_tags::stageLeft.id;
  // int tagOfInterest3 = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                          field_points::blue_alliance::april_tags::stageRight.id :
  //                          field_points::red_alliance::april_tags::stageRight.id;
  // const auto targetValues = GetSeeingCamera();
  // if (targetValues && (tagOfInterest1 == targetValues.value().tagID || tagOfInterest2 == targetValues.value().tagID ||
  //                      tagOfInterest3 == targetValues.value().tagID)) {
  //   return static_cast<units::degree_t>(targetValues.value().tagPose.Rotation().Z());
  // } else {
  return std::nullopt;
  // }
}

void VisionSubsystem::SetPipeline(uint16_t tag) {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable(primaryCameraTableName);

  if constexpr (feature_flags::nt_debugging) {
    frc::SmartDashboard::PutNumber("(Vision) Setting Pipeline", tag);
  }

  table->PutNumber("pipeline", tag);
}

void VisionSubsystem::RequestFilterReset() {
  m_cameraInterface.RequestTargetFilterReset();
}

LimelightTarget::tValues VisionSubsystem::GetPrimaryCameraTargetValues() {
  return m_cameraInterface.m_target.GetTarget(true, primaryCameraTableName);
}

LimelightTarget::tValues VisionSubsystem::GetSecondaryCameraTargetValues() {
  return m_cameraInterface.m_target.GetTarget(true, secondaryCameraTableName);
}

std::optional<whichCamera> VisionSubsystem::getWhichCamera(bool forFeeder) {
  // int tagOfInterest = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                         field_points::blue_alliance::april_tags::speakerCenter.id :
  //                         field_points::red_alliance::april_tags::speakerCenter.id;

  // if (forFeeder) {
  //   tagOfInterest = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
  //                       field_points::blue_alliance::april_tags::stageCenter.id :
  //                       field_points::red_alliance::april_tags::stageCenter.id;
  // }

  // if (tagOfInterest == GetPrimaryCameraTargetValues().tagID) {
  //   return whichCamera::PRIMARY_CAMERA;
  // } else if (tagOfInterest == GetSecondaryCameraTargetValues().tagID) {
  //   return whichCamera::SECONDARY_CAMERA;
  // } else {
  return std::nullopt;
  // }
}

std::optional<LimelightTarget::tValues> VisionSubsystem::GetSeeingCamera(bool forFeeder) {
  const auto camera = getWhichCamera(forFeeder);
  if (camera && camera == whichCamera::PRIMARY_CAMERA) {
    return GetPrimaryCameraTargetValues();
  } else if (camera && camera == whichCamera::SECONDARY_CAMERA) {
    return GetSecondaryCameraTargetValues();
  } else {
    return std::nullopt;
  }
}

void VisionSubsystem::Disable() {
  SetPipeline(0);
}

void VisionSubsystem::UpdateYaw(std::stop_token stopToken) {
  while (!stopToken.stop_requested()) {
    const auto latestPose = m_pDriveSubsystem->GetRawOdometry();
    LimelightHelpers::SetRobotOrientation(
        primaryCameraTableName, latestPose.Rotation().Degrees().to<double>(), 0, 0, 0, 0, 0);
    LimelightHelpers::SetRobotOrientation(
        secondaryCameraTableName, latestPose.Rotation().Degrees().to<double>(), 0, 0, 0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds{20});
  }
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
  m_targetPose = frc::Pose3d(frc::Translation3d(units::make_unit<units::meter_t>(tagPoseCamSpace.at(0)),
                                                units::make_unit<units::meter_t>(tagPoseCamSpace.at(1)),
                                                units::make_unit<units::meter_t>(tagPoseCamSpace.at(2))),
                             frc::Rotation3d(units::make_unit<units::radian_t>(tagPoseCamSpace.at(3)),
                                             units::make_unit<units::radian_t>(tagPoseCamSpace.at(4)),
                                             units::make_unit<units::radian_t>(tagPoseCamSpace.at(5))));

  auto tagId = table->GetNumber("tid", 0.0);

  m_tid = tagId;
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
    m_targetPose.Z() = m_zFilter.Calculate(m_targetPose.Z());

    if constexpr (feature_flags::nt_debugging) {
      frc::SmartDashboard::PutNumber("VisionSubsystem/FilteredPitch (deg)", m_pitch.to<double>());
      frc::SmartDashboard::PutNumber("VisionSubsystem/FilteredYaw (deg)", m_yaw.to<double>());
    }
  }

  m_area = (table->GetNumber("ta", 0.0));
  m_totalLatency = units::make_unit<units::millisecond_t>(rawRobotPose.at(6));

  return tValues{
      m_robotPose, m_robotPoseWPI, m_targetPose, m_hasTargets, m_pitch, m_yaw, m_area, m_tid, m_totalLatency};
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
    m_zFilter.Calculate(currentValue.tagPose.Z());
  }
}
