/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/filter/LinearFilter.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <wpi/DataLog.h>

#include <stop_token>
#include <string>
#include <thread>

#include "argos_lib/config/config_types.h"
#include "argos_lib/general/interpolation.h"
#include "argos_lib/general/nt_subscriber.h"
#include "argos_lib/general/odometry_aim.h"
#include "constants/interpolation_maps.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
#include "swerve_drive_subsystem.h"

enum class whichCamera { LEFT_CAMERA = 0, RIGHT_CAMERA };

class LimelightTarget {
 private:
  frc::Pose3d m_robotPose;              ///< 3d pose of robot relative to field center
  frc::Pose3d m_robotPoseWPI;           ///< 3d pose of robot relative to WPI reference for active alliance
  frc::Pose3d m_targetPose;             ///< 3d pose of Target relative to camera (Z - Forward, X - right, Y - down)
  bool m_hasTargets;                    ///< True if the camera has a target it can read
  units::degree_t m_pitch;              ///< Pitch of target relative to camera -24.85 to 24.85 degrees
  units::degree_t m_yaw;                ///< Yaw of target relative to camera -31.65 to 31.65 degrees
  double m_area;                        ///< Area of the target in percentage of total pixels
  units::millisecond_t m_totalLatency;  ///< Total latency
  frc::LinearFilter<units::degree_t> m_txFilter;
  frc::LinearFilter<units::degree_t> m_tyFilter;
  frc::LinearFilter<units::meter_t> m_zFilter;
  bool m_resetFilterFlag;
  double m_tid;  ///< Tag ID

 public:
  LimelightTarget()
      : m_txFilter{frc::LinearFilter<units::degree_t>::SinglePoleIIR(0.01, 0.02_s)}
      , m_tyFilter{frc::LinearFilter<units::degree_t>::SinglePoleIIR(0.01, 0.02_s)}
      , m_zFilter{frc::LinearFilter<units::meter_t>::SinglePoleIIR(0.01, 0.02_s)}
      , m_resetFilterFlag{false} {}

  /**
   * @brief Wraps members of LimelightTarget for use elsewhere
   *
   */
  struct tValues {
    frc::Pose3d robotPose;              ///< @copydoc LimelightTarget::m_robotPose
    frc::Pose3d robotPoseWPI;           ///< @copydoc LimelightTarget::m_robotPoseWPI
    frc::Pose3d tagPose;                ///< @copydoc LimelightTarget::m_targetPose
    bool hasTargets;                    ///< @copydoc LimelightTarget::m_hasTargets
    units::degree_t m_pitch;            ///< @copydoc LimelightTarget::m_pitch
    units::degree_t m_yaw;              ///< @copydoc LimelightTarget::m_yaw
    double m_area;                      ///< @copydoc LimelightTarget::m_area
    double tagID;                       ///< @copydoc LimelightTarget::m_tid
    units::millisecond_t totalLatency;  ///< @copydoc LimelightTarget::m_totalLatency
  };

  /**
   * @brief Get the values of the camera's current target
   *
   * @return tValues
   */
  tValues GetTarget(bool filter, std::string cameraName);

  /**
   * @brief Does the camera see a target?
   *
   * @return true - The camera does see a target
   * @return false - The camera does not see a target
   */
  bool HasTarget();

  void ResetFilters(std::string cameraName);

  void ResetOnNextTarget();
};

/**
 * @brief Provides methods for interacting with the camera on a high level
 *
 */
class CameraInterface {
 public:
  CameraInterface();

  LimelightTarget m_target;  ///< object that holds the current target seen by the camera

  /**
   * @brief Get the closest target the camera can see CAN RETURN NONE
   *
   * @return std::optional<photonlib::PhotonTrackedTarget>
   */
  std::optional<LimelightTarget> GetClosestTarget();

  /**
   * @brief Turns the camera's driver mode on and off
   *
   * @param mode True is drive control. False is no drive control
   */
  void SetDriverMode(bool mode);

  void RequestTargetFilterReset();
};

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem(const argos_lib::RobotInstance instance, SwerveDriveSubsystem* pDriveSubsystem);

  enum class InterpolationMode { LinearInterpolation, Polynomial, Trig };

  /**
   * @brief Get the robot poses and latencies from left camera
   *
   * @return LimelightTarget::tValues
   */
  [[nodiscard]] LimelightTarget::tValues GetLeftCameraTargetValues();

  /**
   * @brief Get the robot poses and latencies from right camera
   *
   * @return LimelightTarget::tValues
   */
  [[nodiscard]] LimelightTarget::tValues GetRightCameraTargetValues();

  void SetPipeline(uint16_t tag);

  void RequestFilterReset();

  void SetAimWhileMove(bool val);
  [[nodiscard]] bool IsAimWhileMoveActive();

  void SetOdometryAiming(bool val);
  [[nodiscard]] bool IsOdometryAimingActive();

  void SetEnableStaticRotation(bool val);
  [[nodiscard]] bool IsStaticRotationEnabled();

  std::optional<LimelightTarget::tValues> GetSeeingCamera();
  std::optional<whichCamera> getWhichCamera();
  std::optional<frc::Pose3d> GetClosestReefTagPose();
  void SetLeftAlign(bool val);
  void SetRightAlign(bool val);
  [[nodiscard]] bool LeftAlignmentRequested();
  [[nodiscard]] bool RightAlignmentRequested();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /// @brief it disables (duh)
  void Disable();

 private:
  const std::string leftCameraTableName = "/limelight-hileft";
  const std::string rightCameraTableName = "/limelight-hiright";

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  CameraInterface m_cameraInterface;  ///< Interface to limelight camera

  argos_lib::RobotInstance
      m_instance;  ///< Contains either the competition bot or practice bot. Differentiates between the two
  SwerveDriveSubsystem* m_pDriveSubsystem;     ///< Pointer to drivetrain for reading some odometry
  LimelightTarget::tValues m_oldTargetValues;  ///< The old robot poses and latencies
  bool m_usePolynomial;                        ///< specifies whether to use the polynomial to obtain shooter angle
  bool m_useTrigonometry;                      ///< specifies whether to use the trigonometry to obtain shooter angle
  bool m_isAimWhileMoveActive;                 ///< true if aiming trigger is pressed and locked
  bool m_enableStaticRotation;                 ///< true if you want to rotate in the absence of translation input
  bool m_isOdometryAimingActive;               ///< true if we want to aim without vision
  bool m_isLeftAlignActive;                    ///< true if left alignment is requested
  bool m_isRightAlignActive;                   ///< true if right alignment is requested
  argos_lib::NTSubscriber m_leftCameraFrameUpdateSubscriber;   ///< Subscriber to manage all updates from left camera
  argos_lib::NTSubscriber m_rightCameraFrameUpdateSubscriber;  ///< Subscriber to manage all updates from right camera
  std::jthread m_yawUpdateThread;

  wpi::log::StructLogEntry<frc::Pose2d> m_leftCameraMegaTag2PoseLogger;
  wpi::log::StructLogEntry<frc::Pose2d> m_rightCameraMegaTag2PoseLogger;

  void UpdateYaw(std::stop_token stopToken);
};
