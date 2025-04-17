/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <choreo/trajectory/SwerveSample.h>
#include <frc/Timer.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <wpi/DataLog.h>

#include <array>
#include <memory>
#include <mutex>
#include <thread>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "argos_lib/config/config_types.h"
#include "argos_lib/general/nt_motor_pid_tuner.h"
#include "argos_lib/general/nt_subscriber.h"
#include "argos_lib/homing/fs_homing.h"
#include "constants/feature_flags.h"
#include "ctre/phoenix6/sim/CANcoderSimState.hpp"
#include "ctre/phoenix6/sim/TalonFXSimState.hpp"
#include "frc/StateSpaceUtil.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Transform3d.h"

class SwerveModule {
 public:
  // MOTORS
  ctre::phoenix6::hardware::TalonFX m_drive;
  ctre::phoenix6::hardware::TalonFX m_turn;
  // ENCODER
  ctre::phoenix6::hardware::CANcoder m_encoder;

  /**
   * @brief Construct a new Swerve Module object
   *
   * @param driveAddr address of the drive motor on the module
   * @param turnAddr address of the turn motor on the module
   * @param encoderAddr address of the encoder on this module
   */
  SwerveModule(const argos_lib::CANAddress& driveAddr,
               const argos_lib::CANAddress& turnAddr,
               const argos_lib::CANAddress& encoderAddr);

  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();
  void SimulationPeriodic(const frc::SwerveModuleState& desiredState, units::second_t dt);

 private:
  // Internal state to track simulated drive position (in sensor units).
  units::turn_t m_simDrivePos = 0.0_tr;
};

/**
 * @brief Subsystem for controlling the swerve drivetrain of the robot
 *
 */
class SwerveDriveSubsystem : public frc2::SubsystemBase {
 public:
  /**
   * @brief Enumerator for either field-centric or robot centric drive modes.
   *
   */
  enum DriveControlMode { fieldCentricControl, robotCentricControl };

  explicit SwerveDriveSubsystem(const argos_lib::RobotInstance instance);

  virtual ~SwerveDriveSubsystem();

  void SimulationPeriodic() override;

  /**
   * @brief Handle the robot disabling
   */
  void Disable();

  /**
   * @brief Main drive function for the robot
   *
   * @param fwVelocity Percent speed forward.  Range [-1.0, 1.0] where positive 1.0 is full speed forward
   * @param sideVelocity Percent speed perpendicular to the robots front.  Range [-1.0, 1.0] where positive 1.0 is full speed left
   * @param rotVelocity Percent speed of rotation of the chassis.  Range [-1.0, 1.0] where positive 1.0 is full speed counterclockwise
   */
  void SwerveDrive(const double fwVelocity, const double sideVelocity, const double rotVelocity);

  /// @brief Same as polar swerve drive function, but also takes in a rotational velocity to apply ONLY USE IN FIELD-CENTRIC
  /// @param velAngle Angle of velocity vector, [0, 360] with 0 degrees being field-centric home
  /// @param velocity Magnitude of velocity on [0, 1] to apply
  /// @param rotVelocity Percent speed of rotation of the chassis.  Range [-1.0, 1.0] where positive 1.0 is full speed counterclockwise
  void SwerveDrive(const units::degree_t& velAngle, const double& velocity, const double& rotVelocity);

  /// @brief Takes in speeds as a polar vector, and calculates the forward and side velocity to apply ONLY USE IN FIELD-CENTRIC
  /// @param velAngle Angle of velocity vector, [0, 360] with 0 degrees being field-centric home
  /// @param velocity Magnitude of velocity on [0, 1] to apply
  void SwerveDrive(const units::degree_t& velAngle, const double& velocity);

  /// @brief Drive a set chassis speed.  Used for choreo path tracking
  /// @param desiredChassisSpeed Motion parameters
  void SwerveDrive(frc::ChassisSpeeds desiredChassisSpeed);

  /// @brief Follow choreo path
  /// @param sample Desired state for robot
  void SwerveDrive(const choreo::SwerveSample& sample);

  /**
   * @brief Stop all motors
   */
  void StopDrive();

  /// @todo Use 2025 ChoreoLib
  // choreolib::ChoreoControllerFunction GetChoreoControllerFunction();

  /**
   * @brief Save homes to persistent storage and updates module motors
   *
   * @param angle Current angle of all the swerve modules.  They should all be oriented the same during homing.
   *
   */
  void Home(const units::degree_t& angle);

  /**
   * @brief Tell the robot it's in it's correct field-oriented "Front"
   *
   * @param homeAngle Current orientation of the robot
   * @param updateOdometry Also update odometry field-centric angle
   */
  void FieldHome(units::degree_t homeAngle = 0_deg, bool updateOdometry = true);

  void FlipFieldHome();

  /**
   * @brief Set current robot position.  Useful for initializing initial position before autonomous
   *
   * @param currentPose Field-centric pose of the robot
   */
  void InitializeOdometry(const frc::Pose2d& currentPose);

  frc::Rotation2d GetContinuousOdometryAngle();

  frc::Rotation2d GetRawOdometryAngle();

  frc::Rotation2d GetNearestSquareAngle();

  frc::Pose2d GetContinuousOdometry();

  frc::Pose2d GetRawOdometry();

  /**
   * @brief Reads module states & gyro and updates pose estimator.
   */
  void UpdateEstimatedPose();

  /**
   * @brief Get the field-centric angle of the robot based on gyro and saved reference orientation
   *
   * @return Field-centric angle of the robot where 0 degrees is intake oriented toward
   *         opposing alliance operator station positive CCW.
   */
  units::degree_t GetFieldCentricAngle();

  /**
   * @brief Get the latest pose estimate
   *
   * @return Latest pose
   */
  frc::Pose2d GetPoseEstimate(const frc::Pose2d& robotPose, const units::millisecond_t& latency);

  void UpdateVisionMeasurement(const frc::Pose2d& poseEstimate,
                               units::second_t timestamp,
                               const wpi::array<double, 3>& visionMeasurementStdDevs);

  void SetControlMode(SwerveDriveSubsystem::DriveControlMode controlMode);

  /**
   * @brief Initialize motors from persistent storage
   *
   */
  void InitializeMotors();

  /**
   * @brief Change PID parameters for linear follower.  These adjust velocities based on distance
   *        error from path goal
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   */
  void UpdateFollowerLinearPIDParams(double kP, double kI, double kD);

  /**
   * @brief Change PID parameters for rotational follower.  These adjust velocities based on angle
   *        error from path goal
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   */
  void UpdateFollowerRotationalPIDParams(double kP, double kI, double kD);

  /**
   * @brief Cancel the current driving profile without marking it complete
   */
  void CancelDrivingProfile();

  /**
   * @brief Check if a driving profile path has been completed
   *
   * @return true when a path is completed and not canceled
   */
  bool ProfileIsComplete() const;

  /**
   * @brief Check if drivetrain is following a profile
   *
   * @return true when robot is following profile
   */
  bool IsFollowingProfile() const;

  /**
   * @brief Get the robot velocity in chassis frame (x toward intake, y toward left) based on
   *        GetCurrentModuleStates() output
   *
   * @return frc::ChassisSpeeds Velocity based on module states
   */
  frc::ChassisSpeeds GetChassisVelocity();

  /**
   * @brief Put the robot wheels in an x shape where it locks the movement of it
   */
  void LockWheels();

  bool GetManualOverride();

  /**
   * @brief Get the robot pitch as determined by the pigeon IMU
   *
   * @return pitch in unit degrees
   */
  units::degree_t GetRobotPitch() { return m_pigeonIMU.GetRoll().GetValue(); }

  /**
   * @brief Get the rate of robot pitch
   *
   * @return pitch rate in unit degrees per second
   */
  units::degrees_per_second_t GetRobotPitchRate();

  units::degree_t GetIMUYaw();
  units::degrees_per_second_t GetIMUYawRate();

  void SimDrive();

  void SetTrajectoryDisplay(const std::vector<frc::Pose2d>& trajectory);

 private:
  argos_lib::RobotInstance m_instance;

  DriveControlMode m_controlMode;  ///< Active control mode

  SwerveModule m_frontLeft;   ///< Front left swerve module
  SwerveModule m_frontRight;  ///< Front right swerve module
  SwerveModule m_backRight;   ///< Back right swerve module
  SwerveModule m_backLeft;    ///< Back left swerve module

  // GYROSCOPIC SENSORS
  ctre::phoenix6::hardware::Pigeon2 m_pigeonIMU;

  units::degree_t m_fieldHomeOffset;  ///< Offset from IMU angle to 0 field angle (intake away from driver station)

  /**
 * @brief A struct for holding the 3 different input velocities, for organization
 *
 */
  struct Velocities {
    const double fwVelocity;
    const double sideVelocity;
    const double rotVelocity;
  };

  frc::SwerveDriveKinematics<4> m_swerveDriveKinematics;  ///< Kinematics model for swerve drive system

  frc::SwerveDriveOdometry<4> m_odometry;      ///< Odometry to track robot
  units::degree_t m_prevOdometryAngle;         ///< Last odometry angle used for continuous calculations
  units::degree_t m_continuousOdometryOffset;  ///< Offset to convert [-180,180] odometry angle to continuous angle

  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;  ///< accounts vision-based measurements for odometry
  std::mutex m_poseEstimatorLock;
  std::thread m_odometryThread;  ///< Updates robot odometry at very high rate
  std::chrono::time_point<std::chrono::steady_clock>
      m_odometryResetTime;  ///< Time when odometry was last reset to known position
  bool m_stillRunning;      ///< false indicates subsystem is being destroyed

  // std::FILE SYSTEM HOMING STORAGE
  argos_lib::SwerveFSHomingStorage m_fsStorage;  ///< Roborio filesystem access for homes

  bool m_followingProfile;  ///< True when an incomplete drive profile is being run
  bool m_profileComplete;   ///< True once a drive profile has been completed
  bool m_manualOverride;
  std::chrono::time_point<std::chrono::steady_clock> m_swerveProfileStartTime;  ///< Time when active profile began
  frc::PIDController m_xPID;      ///< Correction parameters for x error when following drive profile
  frc::PIDController m_yPID;      ///< Correction parameters for y error when following drive profile
  frc::PIDController m_thetaPID;  ///< Correction parameters for y error when following drive profile

  std::unique_ptr<argos_lib::NTMotorPIDTuner> m_driveMotorPIDTuner;  ///< Utility to tune drive motors
  std::unique_ptr<argos_lib::NTSubscriber> m_linearFollowerTuner_P;
  std::unique_ptr<argos_lib::NTSubscriber> m_linearFollowerTuner_I;
  std::unique_ptr<argos_lib::NTSubscriber> m_linearFollowerTuner_D;
  std::unique_ptr<argos_lib::NTSubscriber> m_rotationalFollowerTuner_P;
  std::unique_ptr<argos_lib::NTSubscriber> m_rotationalFollowerTuner_I;
  std::unique_ptr<argos_lib::NTSubscriber> m_rotationalFollowerTuner_D;

  wpi::array<frc::SwerveModuleState, 4> GetRawModuleStates(frc::ChassisSpeeds velocities);
  /**
 * @brief Get the Raw Module States object and switch between robot-centric and field-centric
 *
 * @param velocities Desired velocity
 * @return wpi::array<frc::SwerveModuleState, 4>
 */
  wpi::array<frc::SwerveModuleState, 4> GetRawModuleStates(SwerveDriveSubsystem::Velocities velocities);

  /**
   * @brief Get the active states of all swerve modules
   *
   * @return Active module states
   */
  wpi::array<frc::SwerveModuleState, 4> GetCurrentModuleStates();

  /**
   * @brief Get the active positions of all swerve modules
   *
   * @return Active module positions
   */
  wpi::array<frc::SwerveModulePosition, 4> GetCurrentModulePositions();

  /**
   * @brief Save homes to a file
   *
   * @param angle Current angle of all the swerve modules.  They should all be oriented the same during homing.
   *
   */
  void HomeToFS(const units::degree_t& angle);

  /**
   * @brief Initialize motors from saved file
   *
   */
  void InitializeMotorsFromFS();

  void ResetIMUYaw();

  wpi::array<frc::SwerveModuleState, 4> OptimizeAllModules(wpi::array<frc::SwerveModuleState, 4> rawStates);

  void ClosedLoopDrive(wpi::array<frc::SwerveModuleState, 4> moduleStates);

  wpi::log::StructLogEntry<frc::Pose2d> m_poseEstimateLogger;
  wpi::log::StructArrayLogEntry<frc::SwerveModuleState> m_setpointLogger;
  wpi::log::StructArrayLogEntry<frc::SwerveModuleState> m_stateLogger;

  struct SimVelocities {
    double fwVelocity;
    double sideVelocity;
    double rotVelocity;
  };
  SimVelocities m_simVelocities;

  double m_simulatedHeading;

  // Field2D for visualization
  frc::Field2d m_field;
};
