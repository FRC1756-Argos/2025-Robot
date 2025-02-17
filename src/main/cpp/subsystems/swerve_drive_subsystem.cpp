/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/swerve_drive_subsystem.h"

#include <argos_lib/config/cancoder_config.h>
#include <argos_lib/config/falcon_config.h>
#include <argos_lib/general/angle_utils.h>
#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/frequency.h>
#include <units/velocity.h>

#include <memory>

#include "Constants.h"
#include "utils/sensor_conversions.h"

using namespace argos_lib::swerve;
using argos_lib::angle::ConstrainAngle;
using ctre::phoenix6::controls::DutyCycleOut;
using ctre::phoenix6::controls::PositionVoltage;
using ctre::phoenix6::controls::VelocityVoltage;
using ctre::phoenix6::hardware::CANcoder;
using ctre::phoenix6::hardware::Pigeon2;
using ctre::phoenix6::hardware::TalonFX;

SwerveDriveSubsystem::SwerveDriveSubsystem(const argos_lib::RobotInstance instance)
    : m_instance(instance)
    , m_controlMode(SwerveDriveSubsystem::DriveControlMode::fieldCentricControl)
    , m_frontLeft(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::frontLeftDrive :
                                                                      address::practice_bot::drive::frontLeftDrive,
                  instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::frontLeftTurn :
                                                                      address::practice_bot::drive::frontLeftTurn,
                  instance == argos_lib::RobotInstance::Competition ? address::comp_bot::encoders::frontLeftEncoder :
                                                                      address::practice_bot::encoders::frontLeftEncoder)
    , m_frontRight(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::frontRightDrive :
                                                                       address::practice_bot::drive::frontRightDrive,
                   instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::frontRightTurn :
                                                                       address::practice_bot::drive::frontRightTurn,
                   instance == argos_lib::RobotInstance::Competition ?
                       address::comp_bot::encoders::frontRightEncoder :
                       address::practice_bot::encoders::frontRightEncoder)
    , m_backRight(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::backRightDrive :
                                                                      address::practice_bot::drive::backRightDrive,
                  instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::backRightTurn :
                                                                      address::practice_bot::drive::backRightTurn,
                  instance == argos_lib::RobotInstance::Competition ? address::comp_bot::encoders::backRightEncoder :
                                                                      address::practice_bot::encoders::backRightEncoder)
    , m_backLeft(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::backLeftDrive :
                                                                     address::practice_bot::drive::backLeftDrive,
                 instance == argos_lib::RobotInstance::Competition ? address::comp_bot::drive::backLeftTurn :
                                                                     address::practice_bot::drive::backLeftTurn,
                 instance == argos_lib::RobotInstance::Competition ? address::comp_bot::encoders::backLeftEncoder :
                                                                     address::practice_bot::encoders::backLeftEncoder)
    , m_pigeonIMU(instance == argos_lib::RobotInstance::Competition ? address::comp_bot::sensors::pigeonIMU.address :
                                                                      address::practice_bot::sensors::pigeonIMU.address,
                  std::string(instance == argos_lib::RobotInstance::Competition ?
                                  address::comp_bot::sensors::pigeonIMU.busName :
                                  address::practice_bot::sensors::pigeonIMU.busName))
    , m_swerveDriveKinematics(
          // Forward is positive X, left is positive Y
          // Front Left
          frc::Translation2d(measure_up::chassis::length / 2 - measure_up::swerve_offsets::frontLeftLOffset,
                             measure_up::chassis::width / 2 - measure_up::swerve_offsets::frontLeftWOffset),
          // Front Right
          frc::Translation2d(measure_up::chassis::length / 2 - measure_up::swerve_offsets::frontRightLOffset,
                             -measure_up::chassis::width / 2 + measure_up::swerve_offsets::frontRightWOffset),
          // Back Right
          frc::Translation2d(-measure_up::chassis::length / 2 + measure_up::swerve_offsets::backRightLOffset,
                             -measure_up::chassis::width / 2 + measure_up::swerve_offsets::backRightWOffset),
          // Back Left
          frc::Translation2d(-measure_up::chassis::length / 2 + measure_up::swerve_offsets::backLeftLOffset,
                             measure_up::chassis::width / 2 - measure_up::swerve_offsets::backLeftWOffset))
    , m_odometry{m_swerveDriveKinematics, frc::Rotation2d(GetIMUYaw()), GetCurrentModulePositions(), frc::Pose2d{}}
    , m_prevOdometryAngle{0_deg}
    , m_continuousOdometryOffset{0_deg}
    , m_poseEstimator{m_swerveDriveKinematics, frc::Rotation2d(GetIMUYaw()), GetCurrentModulePositions(), frc::Pose2d{}}
    , m_odometryThread{}
    , m_odometryResetTime{}
    , m_stillRunning{true}
    , m_fsStorage(paths::swerveHomesPath)
    , m_followingProfile(false)
    , m_profileComplete(false)
    , m_manualOverride(false)
    , m_swerveProfileStartTime()
    , m_rotationalPIDConstraints{instance == argos_lib::RobotInstance::Competition ?
                                     frc::TrapezoidProfile<units::radians>::Constraints(
                                         controlLoop::comp_bot::drive::rotational_follower::angularVelocity,
                                         controlLoop::comp_bot::drive::rotational_follower::angularAcceleration) :
                                     frc::TrapezoidProfile<units::radians>::Constraints(
                                         controlLoop::practice_bot::drive::rotational_follower::angularVelocity,
                                         controlLoop::practice_bot::drive::rotational_follower::angularAcceleration)}
    , m_linearPID(instance == argos_lib::RobotInstance::Competition ?
                      frc::PIDController{controlLoop::comp_bot::drive::linear_follower::kP,
                                         controlLoop::comp_bot::drive::linear_follower::kI,
                                         controlLoop::comp_bot::drive::linear_follower::kD} :
                      frc::PIDController{controlLoop::practice_bot::drive::linear_follower::kP,
                                         controlLoop::practice_bot::drive::linear_follower::kI,
                                         controlLoop::practice_bot::drive::linear_follower::kD})
    , m_followerController{m_linearPID,
                           m_linearPID,
                           instance == argos_lib::RobotInstance::Competition ?
                               frc::ProfiledPIDController<units::radians>{
                                   controlLoop::comp_bot::drive::rotational_follower::kP,
                                   controlLoop::comp_bot::drive::rotational_follower::kI,
                                   controlLoop::comp_bot::drive::rotational_follower::kD,
                                   m_rotationalPIDConstraints} :
                               frc::ProfiledPIDController<units::radians>{
                                   controlLoop::practice_bot::drive::rotational_follower::kP,
                                   controlLoop::practice_bot::drive::rotational_follower::kI,
                                   controlLoop::practice_bot::drive::rotational_follower::kD,
                                   m_rotationalPIDConstraints}}
    , m_driveMotorPIDTuner{nullptr}
    , m_linearFollowerTuner_P{nullptr}
    , m_linearFollowerTuner_I{nullptr}
    , m_linearFollowerTuner_D{nullptr}
    , m_rotationalFollowerTuner_P{nullptr}
    , m_rotationalFollowerTuner_I{nullptr}
    , m_rotationalFollowerTuner_D{nullptr}
    , m_rotationalFollowerConstraintTuner_vel{nullptr}
    , m_rotationalFollowerConstraintTuner_accel{nullptr}
    , m_poseEstimateLogger{frc::DataLogManager::GetLog(), "poseEstimate"}
    , m_setpointLogger{frc::DataLogManager::GetLog(), "ModuleSetpoints"}
    , m_stateLogger{frc::DataLogManager::GetLog(), "ModuleStates"} {
  // TURN MOTORS CONFIG
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::frontLeftTurn,
                                         motorConfig::practice_bot::drive::frontLeftTurn>(
      m_frontLeft.m_turn, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::frontRightTurn,
                                         motorConfig::practice_bot::drive::frontRightTurn>(
      m_frontRight.m_turn, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::backRightTurn,
                                         motorConfig::practice_bot::drive::backRightTurn>(
      m_backRight.m_turn, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::backLeftTurn,
                                         motorConfig::practice_bot::drive::backLeftTurn>(
      m_backLeft.m_turn, 100_ms, instance);

  // DRIVE MOTOR CONFIGS
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::genericDrive,
                                         motorConfig::practice_bot::drive::genericDrive>(
      m_frontLeft.m_drive, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::genericDrive,
                                         motorConfig::practice_bot::drive::genericDrive>(
      m_frontRight.m_drive, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::genericDrive,
                                         motorConfig::practice_bot::drive::genericDrive>(
      m_backLeft.m_drive, 100_ms, instance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::drive::genericDrive,
                                         motorConfig::practice_bot::drive::genericDrive>(
      m_backRight.m_drive, 100_ms, instance);

  // CAN ENCODER CONFIG
  argos_lib::cancoder_config::CanCoderConfig<encoder_conf::comp_bot::drive::genericTurn,
                                             encoder_conf::practice_bot::drive::genericTurn>(
      m_frontLeft.m_encoder, 100_ms, instance);
  argos_lib::cancoder_config::CanCoderConfig<encoder_conf::comp_bot::drive::genericTurn,
                                             encoder_conf::practice_bot::drive::genericTurn>(
      m_frontRight.m_encoder, 100_ms, instance);
  argos_lib::cancoder_config::CanCoderConfig<encoder_conf::comp_bot::drive::genericTurn,
                                             encoder_conf::practice_bot::drive::genericTurn>(
      m_backRight.m_encoder, 100_ms, instance);
  argos_lib::cancoder_config::CanCoderConfig<encoder_conf::comp_bot::drive::genericTurn,
                                             encoder_conf::practice_bot::drive::genericTurn>(
      m_backLeft.m_encoder, 100_ms, instance);

  InitializeMotors();

  const auto odometryUpdateFrequency = 200_Hz;
  frc::SmartDashboard::PutData("Field", &m_field);

  m_pigeonIMU.GetYaw().SetUpdateFrequency(odometryUpdateFrequency);
  m_pigeonIMU.GetAngularVelocityZDevice().SetUpdateFrequency(odometryUpdateFrequency);
  m_frontLeft.m_drive.GetPosition().SetUpdateFrequency(odometryUpdateFrequency);
  m_frontLeft.m_drive.GetVelocity().SetUpdateFrequency(odometryUpdateFrequency);
  m_frontLeft.m_turn.GetPosition().SetUpdateFrequency(odometryUpdateFrequency);
  m_frontLeft.m_turn.GetVelocity().SetUpdateFrequency(odometryUpdateFrequency);
  m_frontRight.m_drive.GetPosition().SetUpdateFrequency(odometryUpdateFrequency);
  m_frontRight.m_drive.GetVelocity().SetUpdateFrequency(odometryUpdateFrequency);
  m_frontRight.m_turn.GetPosition().SetUpdateFrequency(odometryUpdateFrequency);
  m_frontRight.m_turn.GetVelocity().SetUpdateFrequency(odometryUpdateFrequency);
  m_backRight.m_drive.GetPosition().SetUpdateFrequency(odometryUpdateFrequency);
  m_backRight.m_drive.GetVelocity().SetUpdateFrequency(odometryUpdateFrequency);
  m_backRight.m_turn.GetPosition().SetUpdateFrequency(odometryUpdateFrequency);
  m_backRight.m_turn.GetVelocity().SetUpdateFrequency(odometryUpdateFrequency);
  m_backLeft.m_drive.GetPosition().SetUpdateFrequency(odometryUpdateFrequency);
  m_backLeft.m_drive.GetVelocity().SetUpdateFrequency(odometryUpdateFrequency);
  m_backLeft.m_turn.GetPosition().SetUpdateFrequency(odometryUpdateFrequency);
  m_backLeft.m_turn.GetVelocity().SetUpdateFrequency(odometryUpdateFrequency);

  m_followerController.GetThetaController().EnableContinuousInput(-180_deg, 180_deg);

  if constexpr (feature_flags::drive_nt_tuning) {
    m_driveMotorPIDTuner = std::make_unique<argos_lib::NTMotorPIDTuner>(
        "argos/drive/driveMotors",
        std::initializer_list<ctre::phoenix6::hardware::core::CoreTalonFX*>{
            &m_frontLeft.m_drive, &m_frontRight.m_drive, &m_backRight.m_drive, &m_backLeft.m_drive},
        0,
        argos_lib::ClosedLoopSensorConversions{
            argos_lib::GetPositionConversionFactor(sensor_conversions::swerve_drive::drive::ToDistance),
            argos_lib::GetVelocityConversionFactor(sensor_conversions::swerve_drive::drive::ToVelocity),
            argos_lib::GetVelocityConversionFactor(sensor_conversions::swerve_drive::drive::ToVelocity)});
    m_linearFollowerTuner_P = std::make_unique<argos_lib::NTSubscriber>("argos/drive/linearFollower");
    m_linearFollowerTuner_I = std::make_unique<argos_lib::NTSubscriber>("argos/drive/linearFollower");
    m_linearFollowerTuner_D = std::make_unique<argos_lib::NTSubscriber>("argos/drive/linearFollower");
    m_rotationalFollowerTuner_P = std::make_unique<argos_lib::NTSubscriber>("argos/drive/rotationalFollower");
    m_rotationalFollowerTuner_I = std::make_unique<argos_lib::NTSubscriber>("argos/drive/rotationalFollower");
    m_rotationalFollowerTuner_D = std::make_unique<argos_lib::NTSubscriber>("argos/drive/rotationalFollower");
    m_rotationalFollowerConstraintTuner_vel =
        std::make_unique<argos_lib::NTSubscriber>("argos/drive/rotationalFollower");
    m_rotationalFollowerConstraintTuner_accel =
        std::make_unique<argos_lib::NTSubscriber>("argos/drive/rotationalFollower");
    m_linearFollowerTuner_P->AddMonitor(
        "kP",
        [this](double val) {
          m_followerController.GetXController().SetP(val);
          m_followerController.GetYController().SetP(val);
        },
        m_followerController.GetXController().GetP());
    m_linearFollowerTuner_I->AddMonitor(
        "kI",
        [this](double val) {
          m_followerController.GetXController().SetI(val);
          m_followerController.GetYController().SetI(val);
        },
        m_followerController.GetXController().GetI());
    m_linearFollowerTuner_D->AddMonitor(
        "kD",
        [this](double val) {
          m_followerController.GetXController().SetD(val);
          m_followerController.GetYController().SetD(val);
        },
        m_followerController.GetXController().GetD());
    m_rotationalFollowerTuner_P->AddMonitor(
        "kP",
        [this](double val) { m_followerController.GetThetaController().SetP(val); },
        m_followerController.GetThetaController().GetP());
    m_rotationalFollowerTuner_I->AddMonitor(
        "kI",
        [this](double val) { m_followerController.GetThetaController().SetI(val); },
        m_followerController.GetThetaController().GetI());
    m_rotationalFollowerTuner_D->AddMonitor(
        "kD",
        [this](double val) { m_followerController.GetThetaController().SetD(val); },
        m_followerController.GetThetaController().GetD());
    m_rotationalFollowerConstraintTuner_vel->AddMonitor(
        "maxVel (deg_per_s)",
        [this](double val) {
          m_rotationalPIDConstraints.maxVelocity = units::degrees_per_second_t(val);
          UpdateFollowerRotationalPIDConstraints(m_rotationalPIDConstraints);
        },
        units::degrees_per_second_t{m_rotationalPIDConstraints.maxVelocity}.to<double>());
    m_rotationalFollowerConstraintTuner_accel->AddMonitor(
        "maxAccel (deg_per_s^2)",
        [this](double val) {
          m_rotationalPIDConstraints.maxAcceleration = units::degrees_per_second_squared_t(val);
          UpdateFollowerRotationalPIDConstraints(m_rotationalPIDConstraints);
        },
        units::degrees_per_second_squared_t{m_rotationalPIDConstraints.maxAcceleration}.to<double>());
  }

  m_odometryThread = std::thread(&SwerveDriveSubsystem::UpdateEstimatedPose, this);

  frc::DataLogManager::GetLog().AddStructSchema<frc::Pose2d>();
  frc::DataLogManager::GetLog().AddStructSchema<std::array<frc::SwerveModuleState, 4>>();
}

SwerveDriveSubsystem::~SwerveDriveSubsystem() {
  m_stillRunning = false;
  m_odometryThread.join();
}

void SwerveDriveSubsystem::Disable() {
  m_controlMode = DriveControlMode::fieldCentricControl;
  CancelDrivingProfile();
  StopDrive();
}

void SwerveDriveSubsystem::SimulationPeriodic() {
  m_controlMode = DriveControlMode::robotCentricControl;
  SimDrive();
}

void SwerveDriveSubsystem::SimDrive() {
  constexpr double dt = 0.02;
  // Read raw simulated input velocities (range: -1 to 1)
  auto ySpeedRaw = m_simVelocities.fwVelocity;
  auto xSpeedRaw = m_simVelocities.sideVelocity;
  auto rotSpeedRaw = m_simVelocities.rotVelocity;

  // Scale raw input to realistic speeds
  auto ySpeed = (ySpeedRaw * speeds::simDrive::speedScaling);
  auto xSpeed = (xSpeedRaw * speeds::simDrive::speedScaling);
  auto rotSpeed = (rotSpeedRaw * speeds::simDrive::rotationScaling);

  frc::SmartDashboard::PutNumber("Sim X Speed", xSpeed);
  frc::SmartDashboard::PutNumber("Sim Y Speed", ySpeed);
  frc::SmartDashboard::PutNumber("Sim Rotate Speed", rotSpeed);

  // Initialize moduleStates using our stored simulation encoder distances.
  wpi::array<frc::SwerveModulePosition, 4> moduleStates{
      {frc::SwerveModulePosition{units::meter_t(m_encoderSims[0].GetDistance()), frc::Rotation2d(0_deg)},
       frc::SwerveModulePosition{units::meter_t(m_encoderSims[1].GetDistance()), frc::Rotation2d(0_deg)},
       frc::SwerveModulePosition{units::meter_t(m_encoderSims[2].GetDistance()), frc::Rotation2d(0_deg)},
       frc::SwerveModulePosition{units::meter_t(m_encoderSims[3].GetDistance()), frc::Rotation2d(0_deg)}}};

  // Compute desired swerve module states based on chassis speeds.
  SwerveDriveSubsystem::Velocities velocities{ySpeed, xSpeed, rotSpeed};
  auto swerveModuleStates = GetRawModuleStates(velocities);

  {
    double deltaDistanceFL = swerveModuleStates.at(indexes::swerveModules::frontLeftIndex).speed.value() * dt;
    double newDistanceFL = m_encoderSims[0].GetDistance() + deltaDistanceFL;
    m_encoderSims[0].SetDistance(newDistanceFL);
    moduleStates[0] = frc::SwerveModulePosition{units::meter_t(newDistanceFL),
                                                swerveModuleStates.at(indexes::swerveModules::frontLeftIndex).angle};
    frc::SmartDashboard::PutNumber("[sim] Swerve/Module 0 Position", newDistanceFL);
    auto distanceInInchesFL = units::inch_t(newDistanceFL);
    auto sensorPosFL = sensor_conversions::swerve_drive::drive::ToSensorPosition(distanceInInchesFL);
    m_frontLeft.m_drive.SetPosition(sensorPosFL, 20_ms);

    double deltaDistanceFR = swerveModuleStates.at(indexes::swerveModules::frontRightIndex).speed.value() * dt;
    double newDistanceFR = m_encoderSims[1].GetDistance() + deltaDistanceFR;
    m_encoderSims[1].SetDistance(newDistanceFR);
    moduleStates[1] = frc::SwerveModulePosition{units::meter_t(newDistanceFR),
                                                swerveModuleStates.at(indexes::swerveModules::frontRightIndex).angle};
    frc::SmartDashboard::PutNumber("[sim] Swerve/Module 1 Position", newDistanceFR);
    auto distanceInInchesFR = units::inch_t(newDistanceFR);
    auto sensorPosFR = sensor_conversions::swerve_drive::drive::ToSensorPosition(distanceInInchesFR);
    m_frontRight.m_drive.SetPosition(sensorPosFR, 20_ms);

    double deltaDistanceBR = swerveModuleStates.at(indexes::swerveModules::backRightIndex).speed.value() * dt;
    double newDistanceBR = m_encoderSims[2].GetDistance() + deltaDistanceBR;
    m_encoderSims[2].SetDistance(newDistanceBR);
    moduleStates[2] = frc::SwerveModulePosition{units::meter_t(newDistanceBR),
                                                swerveModuleStates.at(indexes::swerveModules::backRightIndex).angle};
    frc::SmartDashboard::PutNumber("[sim] Swerve/Module 2 Position", newDistanceBR);
    auto distanceInInchesBR = units::inch_t(newDistanceBR);
    auto sensorPosBR = sensor_conversions::swerve_drive::drive::ToSensorPosition(distanceInInchesBR);
    m_backRight.m_drive.SetPosition(sensorPosBR, 20_ms);

    double deltaDistanceBL = swerveModuleStates.at(indexes::swerveModules::backLeftIndex).speed.value() * dt;
    double newDistanceBL = m_encoderSims[3].GetDistance() + deltaDistanceBL;
    m_encoderSims[3].SetDistance(newDistanceBL);
    moduleStates[3] = frc::SwerveModulePosition{units::meter_t(newDistanceBL),
                                                swerveModuleStates.at(indexes::swerveModules::backLeftIndex).angle};
    frc::SmartDashboard::PutNumber("[sim] Swerve/Module 3 Position", newDistanceBL);
    auto distanceInInchesBL = units::inch_t(newDistanceBL);
    auto sensorPosBL = sensor_conversions::swerve_drive::drive::ToSensorPosition(distanceInInchesBL);
    m_backLeft.m_drive.SetPosition(sensorPosBL, 20_ms);
  }

  // Update simulated gyro angle:
  double angularVelocityDeg = rotSpeed * (180.0 / 3.14159265358);
  double simulatedRotation = m_gyroSim.GetAngle() + angularVelocityDeg * dt;
  m_gyroSim.SetAngle(simulatedRotation);
  frc::SmartDashboard::PutNumber("[sim] Swerve/Gyro Angle", simulatedRotation);

  // Update the simulated gyro sensor so the odometry thread sees the new value.
  m_pigeonIMU.SetYaw(units::degree_t(simulatedRotation));

  // Update odometry using the updated module states and simulated gyro angle.
  m_odometry.Update(frc::Rotation2d(units::degree_t(simulatedRotation)), moduleStates);
  m_field.SetRobotPose(m_odometry.GetPose());

  frc::SmartDashboard::PutNumber("[sim] Swerve/Pose X", m_odometry.GetPose().X().value());
  frc::SmartDashboard::PutNumber("[sim] Swerve/Pose Y", m_odometry.GetPose().Y().value());
  frc::SmartDashboard::PutNumber("[sim] Swerve/Pose Rotation", m_odometry.GetPose().Rotation().Degrees().value());
}

// SWERVE DRIVE SUBSYSTEM MEMBER FUNCTIONS

wpi::array<frc::SwerveModuleState, 4> SwerveDriveSubsystem::GetRawModuleStates(frc::ChassisSpeeds velocities) {
  return m_swerveDriveKinematics.ToSwerveModuleStates(velocities);
}

wpi::array<frc::SwerveModuleState, 4> SwerveDriveSubsystem::GetRawModuleStates(
    SwerveDriveSubsystem::Velocities velocities) {
  // IF SPEEDS ZERO, SET MOTORS TO ZERO AND RETURN
  if (velocities.fwVelocity == 0 && velocities.sideVelocity == 0 && velocities.rotVelocity == 0) {
    StopDrive();
    frc::ChassisSpeeds emptySpeeds{units::make_unit<units::velocity::meters_per_second_t>(0),
                                   units::make_unit<units::velocity::meters_per_second_t>(0),
                                   units::make_unit<units::angular_velocity::radians_per_second_t>(0)};

    return GetRawModuleStates(emptySpeeds);
  }

  switch (m_controlMode) {
    case (DriveControlMode::
              fieldCentricControl): {  // Construct speeds with field-relative speeds and current IMU Z angle.
      frc::ChassisSpeeds fieldCentricSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          units::make_unit<units::meters_per_second_t>(velocities.fwVelocity),
          units::make_unit<units::meters_per_second_t>(velocities.sideVelocity),
          units::make_unit<units::angular_velocity::radians_per_second_t>(velocities.rotVelocity),
          frc::Rotation2d(GetFieldCentricAngle()));

      // Return the speeds to consumer
      return GetRawModuleStates(fieldCentricSpeeds);
    }

    case (DriveControlMode::robotCentricControl): {
      // Construct speeds just the same as in the current main drive function
      frc::ChassisSpeeds robotCentricSpeeds{
          units::make_unit<units::velocity::meters_per_second_t>(velocities.fwVelocity),
          units::make_unit<units::velocity::meters_per_second_t>(velocities.sideVelocity),
          units::make_unit<units::angular_velocity::radians_per_second_t>(velocities.rotVelocity)};

      return GetRawModuleStates(robotCentricSpeeds);
    }
  }
  frc::ChassisSpeeds emptySpeeds{units::make_unit<units::velocity::meters_per_second_t>(0),
                                 units::make_unit<units::velocity::meters_per_second_t>(0),
                                 units::make_unit<units::angular_velocity::radians_per_second_t>(0)};

  return GetRawModuleStates(emptySpeeds);
}

wpi::array<frc::SwerveModuleState, 4> SwerveDriveSubsystem::GetCurrentModuleStates() {
  return {m_frontLeft.GetState(), m_frontRight.GetState(), m_backRight.GetState(), m_backLeft.GetState()};
}

wpi::array<frc::SwerveModulePosition, 4> SwerveDriveSubsystem::GetCurrentModulePositions() {
  return {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backRight.GetPosition(), m_backLeft.GetPosition()};
}

void SwerveDriveSubsystem::SwerveDrive(const double fwVelocity, const double sideVelocity, const double rotVelocity) {
  GetContinuousOdometry();
  if (!frc::RobotBase::IsSimulation() && fwVelocity == 0 && sideVelocity == 0 && rotVelocity == 0) {
    if (!m_followingProfile) {
      StopDrive();
      return;
    }
  } else {
    // Manual override
    m_manualOverride = true;
    m_followingProfile = false;
    m_profileComplete = false;
  }

  SwerveDriveSubsystem::Velocities velocities{fwVelocity, sideVelocity, rotVelocity};

  // DEBUG STUFF
  if constexpr (feature_flags::nt_debugging) {
    frc::SmartDashboard::PutNumber("(DRIVETRAIN) fwVelocity", fwVelocity);
    frc::SmartDashboard::PutNumber("(DRIVETRAIN) sideVelocity", sideVelocity);
    frc::SmartDashboard::PutNumber("(DRIVETRAIN) rotVelocity", rotVelocity);
    frc::SmartDashboard::PutNumber("CONTROL MODE", m_controlMode);
    frc::SmartDashboard::PutNumber("IMU PIGEON ANGLE", units::degree_t{m_pigeonIMU.GetYaw().GetValue()}.to<double>());
    frc::SmartDashboard::PutNumber("IMU Pitch Rate", GetRobotPitchRate().to<double>());
  }

  if (frc::RobotBase::IsSimulation()) {
    m_simVelocities.fwVelocity = velocities.fwVelocity;
    m_simVelocities.sideVelocity = velocities.sideVelocity;
    m_simVelocities.rotVelocity = velocities.rotVelocity;
  }

  // SET MODULES BASED OFF OF CONTROL MODE
  auto moduleStates = GetCurrentModuleStates();
  frc::Trajectory::State desiredProfileState;
  moduleStates = GetRawModuleStates(velocities);

  moduleStates = OptimizeAllModules(moduleStates);

  // Give module state values to motors

  if (m_followingProfile) {
    // When following profile, use closed-loop velocity
    ClosedLoopDrive(moduleStates);
  } else {
    // FRONT LEFT
    m_frontLeft.m_drive.SetControl(
        DutyCycleOut(moduleStates.at(indexes::swerveModules::frontLeftIndex).speed.to<double>()));

    m_frontLeft.m_turn.SetControl(PositionVoltage(sensor_conversions::swerve_drive::turn::ToSensorUnit(
        moduleStates.at(indexes::swerveModules::frontLeftIndex).angle.Degrees())));

    // FRONT RIGHT
    m_frontRight.m_drive.SetControl(
        DutyCycleOut(moduleStates.at(indexes::swerveModules::frontRightIndex).speed.to<double>()));

    m_frontRight.m_turn.SetControl(PositionVoltage(sensor_conversions::swerve_drive::turn::ToSensorUnit(
        moduleStates.at(indexes::swerveModules::frontRightIndex).angle.Degrees())));

    // BACK RIGHT
    m_backRight.m_drive.SetControl(
        DutyCycleOut(moduleStates.at(indexes::swerveModules::backRightIndex).speed.to<double>()));

    m_backRight.m_turn.SetControl(PositionVoltage(sensor_conversions::swerve_drive::turn::ToSensorUnit(
        moduleStates.at(indexes::swerveModules::backRightIndex).angle.Degrees())));

    // BACK LEFT
    m_backLeft.m_drive.SetControl(
        DutyCycleOut(moduleStates.at(indexes::swerveModules::backLeftIndex).speed.to<double>()));

    m_backLeft.m_turn.SetControl(PositionVoltage(sensor_conversions::swerve_drive::turn::ToSensorUnit(
        moduleStates.at(indexes::swerveModules::backLeftIndex).angle.Degrees())));
  }

  // DEBUG STUFF
  if constexpr (feature_flags::nt_debugging) {
    frc::SmartDashboard::PutNumber("(DRIVETRAIN) FL speed",
                                   moduleStates.at(indexes::swerveModules::frontLeftIndex).speed.to<double>());
    frc::SmartDashboard::PutNumber(
        "(DRIVETRAIN) FL turn", moduleStates.at(indexes::swerveModules::frontLeftIndex).angle.Degrees().to<double>());

    frc::SmartDashboard::PutNumber("(DRIVETRAIN) FR speed",
                                   moduleStates.at(indexes::swerveModules::frontRightIndex).speed.to<double>());
    frc::SmartDashboard::PutNumber(
        "(DRIVETRAIN) FR turn", moduleStates.at(indexes::swerveModules::frontRightIndex).angle.Degrees().to<double>());

    frc::SmartDashboard::PutNumber("(DRIVETRAIN) BR speed",
                                   moduleStates.at(indexes::swerveModules::backRightIndex).speed.to<double>());
    frc::SmartDashboard::PutNumber(
        "(DRIVETRAIN) BR turn", moduleStates.at(indexes::swerveModules::backRightIndex).angle.Degrees().to<double>());

    frc::SmartDashboard::PutNumber("(DRIVETRAIN) BL speed",
                                   moduleStates.at(indexes::swerveModules::backLeftIndex).speed.to<double>());
    frc::SmartDashboard::PutNumber("(DRIVETRAIN) BL turn",
                                   moduleStates.at(indexes::swerveModules::backLeftIndex).angle.Degrees().to<double>());
  }
}

void SwerveDriveSubsystem::SwerveDrive(const units::degree_t& velAngle,
                                       const double& velocity,
                                       const double& rotVelocity) {
  // Filter/Validate inputs
  units::degree_t filteredVelAngle = argos_lib::angle::ConstrainAngle(velAngle, 0_deg, 360_deg);
  double filteredVel = std::clamp<double>(velocity, -1, 1);
  double filteredRotVelocity = std::clamp<double>(rotVelocity, -1, 1);

  // Calculate velocity components and make field-centric speeds
  double fwVel = filteredVel * (units::math::cos(filteredVelAngle).to<double>());
  double leftVel = filteredVel * (units::math::sin(filteredVelAngle).to<double>());

  SwerveDrive(fwVel, leftVel, filteredRotVelocity);
}

void SwerveDriveSubsystem::SwerveDrive(const units::degree_t& velAngle, const double& velocity) {
  SwerveDrive(velAngle, velocity, 0);
}

void SwerveDriveSubsystem::SwerveDrive(frc::ChassisSpeeds desiredChassisSpeed) {
  m_manualOverride = true;
  auto moduleStates = GetRawModuleStates(desiredChassisSpeed);
  moduleStates = OptimizeAllModules(moduleStates);
  ClosedLoopDrive(moduleStates);
}

void SwerveDriveSubsystem::StopDrive() {
  m_frontLeft.m_drive.Set(0.0);
  m_frontLeft.m_turn.Set(0.0);
  m_frontRight.m_drive.Set(0.0);
  m_frontRight.m_turn.Set(0.0);
  m_backRight.m_drive.Set(0.0);
  m_backRight.m_turn.Set(0.0);
  m_backLeft.m_drive.Set(0.0);
  m_backLeft.m_turn.Set(0.0);
}

/// @todo Use 2025 ChoreoLib
// choreolib::ChoreoControllerFunction SwerveDriveSubsystem::GetChoreoControllerFunction() {
//   return choreolib::Choreo::ChoreoSwerveController(
//       frc::PIDController{m_followerController.GetXController().GetP(),
//                          m_followerController.GetXController().GetI(),
//                          m_followerController.GetXController().GetD()},
//       frc::PIDController{m_followerController.GetYController().GetP(),
//                          m_followerController.GetYController().GetI(),
//                          m_followerController.GetYController().GetD()},
//       frc::PIDController{m_followerController.GetThetaController().GetP(),
//                          m_followerController.GetThetaController().GetI(),
//                          m_followerController.GetThetaController().GetD()});
// }

void SwerveDriveSubsystem::Home(const units::degree_t& angle) {
  HomeToFS(angle);

  // RE-ZERO THE IMU
  ResetIMUYaw();

  // SetPosition expects a value in degrees
  m_frontLeft.m_encoder.SetPosition(angle, 50_ms);
  m_frontRight.m_encoder.SetPosition(angle, 50_ms);
  m_backRight.m_encoder.SetPosition(angle, 50_ms);
  m_backLeft.m_encoder.SetPosition(angle, 50_ms);
}

units::degrees_per_second_t SwerveDriveSubsystem::GetRobotPitchRate() {
  return m_pigeonIMU.GetAngularVelocityXDevice().GetValue();
}

void SwerveDriveSubsystem::LockWheels() {
  m_manualOverride = false;

  auto currentState = GetCurrentModuleStates();

  auto frontLeftPosition = frc::SwerveModuleState{0_mps, 45_deg};
  auto frontRightPosition = frc::SwerveModuleState{0_mps, -45_deg};
  auto backRightPosition = frc::SwerveModuleState{0_mps, 45_deg};
  auto backLeftPosition = frc::SwerveModuleState{0_mps, -45_deg};

  auto desiredFrontLeftState =
      argos_lib::swerve::Optimize(frontLeftPosition, currentState.at(0).angle.Degrees(), 0_rpm, 0_fps, 0_fps);
  auto desiredFrontRightState =
      argos_lib::swerve::Optimize(frontRightPosition, currentState.at(1).angle.Degrees(), 0_rpm, 0_fps, 0_fps);
  auto desiredBackRightState =
      argos_lib::swerve::Optimize(backRightPosition, currentState.at(2).angle.Degrees(), 0_rpm, 0_fps, 0_fps);
  auto desiredBackLeftState =
      argos_lib::swerve::Optimize(backLeftPosition, currentState.at(3).angle.Degrees(), 0_rpm, 0_fps, 0_fps);

  m_frontLeft.m_turn.SetControl(
      PositionVoltage(sensor_conversions::swerve_drive::turn::ToSensorUnit(desiredFrontLeftState.angle.Degrees())));

  m_frontRight.m_turn.SetControl(
      PositionVoltage(sensor_conversions::swerve_drive::turn::ToSensorUnit(desiredFrontRightState.angle.Degrees())));
  m_backRight.m_turn.SetControl(
      PositionVoltage(sensor_conversions::swerve_drive::turn::ToSensorUnit(desiredBackRightState.angle.Degrees())));
  m_backLeft.m_turn.SetControl(
      PositionVoltage(sensor_conversions::swerve_drive::turn::ToSensorUnit(desiredBackLeftState.angle.Degrees())));
}

bool SwerveDriveSubsystem::GetManualOverride() {
  return m_manualOverride;
}

void SwerveDriveSubsystem::FieldHome(units::degree_t homeAngle, bool updateOdometry) {
  m_fieldHomeOffset = -GetIMUYaw() - homeAngle;
  if (updateOdometry) {
    // Update odometry as well
    std::lock_guard lock{m_poseEstimatorLock};
    const auto currentPose = m_poseEstimator.GetEstimatedPosition();
    m_poseEstimator.ResetPosition(
        -GetIMUYaw(), GetCurrentModulePositions(), frc::Pose2d{currentPose.Translation(), frc::Rotation2d(homeAngle)});
    m_prevOdometryAngle = m_poseEstimator.GetEstimatedPosition().Rotation().Degrees();
    m_continuousOdometryOffset = 0_deg;
  }
}

void SwerveDriveSubsystem::InitializeOdometry(const frc::Pose2d& currentPose) {
  m_odometryResetTime = std::chrono::steady_clock::now();
  {
    std::lock_guard lock{m_poseEstimatorLock};
    m_poseEstimator.ResetPosition(-GetIMUYaw(), GetCurrentModulePositions(), currentPose);
    m_prevOdometryAngle = m_poseEstimator.GetEstimatedPosition().Rotation().Degrees();
  }
  m_continuousOdometryOffset = 0_deg;
  // Since we know the position, might as well update the driving orientation as well
  FieldHome(currentPose.Rotation().Degrees(), false);
}

frc::Rotation2d SwerveDriveSubsystem::GetContinuousOdometryAngle() {
  frc::Pose2d latestOdometry;
  { latestOdometry = m_poseEstimator.GetEstimatedPosition(); }

  if (m_prevOdometryAngle > 90_deg && latestOdometry.Rotation().Degrees() < -(90_deg)) {
    m_continuousOdometryOffset += 360_deg;
  } else if (m_prevOdometryAngle < -(90_deg) && latestOdometry.Rotation().Degrees() > 90_deg) {
    m_continuousOdometryOffset -= 360_deg;
  }
  m_prevOdometryAngle = latestOdometry.Rotation().Degrees();

  auto continuousOdometry = frc::Rotation2d{latestOdometry.Rotation().Degrees() + m_continuousOdometryOffset};
  if constexpr (feature_flags::nt_debugging) {
    frc::SmartDashboard::PutNumber("(Odometry) Raw Angle", latestOdometry.Rotation().Degrees().to<double>());
    frc::SmartDashboard::PutNumber("(Odometry) Continuous Angle", continuousOdometry.Degrees().to<double>());
  }

  return continuousOdometry;
}

frc::Rotation2d SwerveDriveSubsystem::GetRawOdometryAngle() {
  std::lock_guard lock{m_poseEstimatorLock};
  return m_poseEstimator.GetEstimatedPosition().Rotation();
}

frc::Rotation2d SwerveDriveSubsystem::GetNearestSquareAngle() {
  const auto currentAngle = GetContinuousOdometryAngle().Degrees();
  return units::math::round(currentAngle / 90) * 90;
}

frc::Pose2d SwerveDriveSubsystem::GetContinuousOdometry() {
  frc::Pose2d discontinuousOdometry;
  {
    std::lock_guard lock{m_poseEstimatorLock};
    discontinuousOdometry = m_poseEstimator.GetEstimatedPosition();
  }

  if constexpr (feature_flags::nt_debugging) {
    frc::SmartDashboard::PutNumber("(Odometry) Current X",
                                   units::inch_t{discontinuousOdometry.Translation().X()}.to<double>());
    frc::SmartDashboard::PutNumber("(Odometry) Current Y",
                                   units::inch_t{discontinuousOdometry.Translation().Y()}.to<double>());
  }
  return frc::Pose2d{discontinuousOdometry.Translation(), GetContinuousOdometryAngle()};
}

frc::Pose2d SwerveDriveSubsystem::GetRawOdometry() {
  frc::Pose2d discontinuousOdometry;
  {
    std::lock_guard lock{m_poseEstimatorLock};
    discontinuousOdometry = m_poseEstimator.GetEstimatedPosition();
  }
  if constexpr (feature_flags::nt_debugging) {
    frc::SmartDashboard::PutNumber("(Odometry) Current X",
                                   units::inch_t{discontinuousOdometry.Translation().X()}.to<double>());
    frc::SmartDashboard::PutNumber("(Odometry) Current Y",
                                   units::inch_t{discontinuousOdometry.Translation().Y()}.to<double>());
  }
  return discontinuousOdometry;
}

void SwerveDriveSubsystem::UpdateEstimatedPose() {
  auto& yawUpdate = m_pigeonIMU.GetYaw();
  auto& yawRateUpdate = m_pigeonIMU.GetAngularVelocityZDevice();
  auto& frontLeftDrivePositionUpdate = m_frontLeft.m_drive.GetPosition();
  auto& frontLeftDriveVelocityUpdate = m_frontLeft.m_drive.GetVelocity();
  auto& frontLeftTurnPositionUpdate = m_frontLeft.m_turn.GetPosition();
  auto& frontLeftTurnVelocityUpdate = m_frontLeft.m_turn.GetVelocity();
  auto& frontRightDrivePositionUpdate = m_frontRight.m_drive.GetPosition();
  auto& frontRightDriveVelocityUpdate = m_frontRight.m_drive.GetVelocity();
  auto& frontRightTurnPositionUpdate = m_frontRight.m_turn.GetPosition();
  auto& frontRightTurnVelocityUpdate = m_frontRight.m_turn.GetVelocity();
  auto& backRightDrivePositionUpdate = m_backRight.m_drive.GetPosition();
  auto& backRightDriveVelocityUpdate = m_backRight.m_drive.GetVelocity();
  auto& backRightTurnPositionUpdate = m_backRight.m_turn.GetPosition();
  auto& backRightTurnVelocityUpdate = m_backRight.m_turn.GetVelocity();
  auto& backLeftDrivePositionUpdate = m_backLeft.m_drive.GetPosition();
  auto& backLeftDriveVelocityUpdate = m_backLeft.m_drive.GetVelocity();
  auto& backLeftTurnPositionUpdate = m_backLeft.m_turn.GetPosition();
  auto& backLeftTurnVelocityUpdate = m_backLeft.m_turn.GetVelocity();

  std::chrono::time_point<std::chrono::steady_clock> logTime{};

  while (m_stillRunning) {
    if (0 == ctre::phoenix6::BaseStatusSignal::WaitForAll(20_ms,
                                                          yawUpdate,
                                                          yawRateUpdate,
                                                          frontLeftDrivePositionUpdate,
                                                          frontLeftDriveVelocityUpdate,
                                                          frontLeftTurnPositionUpdate,
                                                          frontLeftTurnVelocityUpdate,
                                                          frontRightDrivePositionUpdate,
                                                          frontRightDriveVelocityUpdate,
                                                          frontRightTurnPositionUpdate,
                                                          frontRightTurnVelocityUpdate,
                                                          backRightDrivePositionUpdate,
                                                          backRightDriveVelocityUpdate,
                                                          backRightTurnPositionUpdate,
                                                          backRightTurnVelocityUpdate,
                                                          backLeftDrivePositionUpdate,
                                                          backLeftDriveVelocityUpdate,
                                                          backLeftTurnPositionUpdate,
                                                          backLeftTurnVelocityUpdate)) {
      auto yaw = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(yawUpdate, yawRateUpdate);
      auto frontLeftDrivePosition = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          frontLeftDrivePositionUpdate, frontLeftDriveVelocityUpdate);
      auto frontLeftTurnPosition = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          frontLeftTurnPositionUpdate, frontLeftTurnVelocityUpdate);
      auto frontRightDrivePosition = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          frontRightDrivePositionUpdate, frontRightDriveVelocityUpdate);
      auto frontRightTurnPosition = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          frontRightTurnPositionUpdate, frontRightTurnVelocityUpdate);
      auto backRightDrivePosition = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          backRightDrivePositionUpdate, backRightDriveVelocityUpdate);
      auto backRightTurnPosition = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          backRightTurnPositionUpdate, backRightTurnVelocityUpdate);
      auto backLeftDrivePosition = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          backLeftDrivePositionUpdate, backLeftDriveVelocityUpdate);
      auto backLeftTurnPosition = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          backLeftTurnPositionUpdate, backLeftTurnVelocityUpdate);

      auto updateTime = yawUpdate.GetTimestamp().GetTime();
      auto frontLeftModule = frc::SwerveModulePosition{
          sensor_conversions::swerve_drive::drive::ToDistance(frontLeftDrivePosition),
          frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(frontLeftTurnPosition)}};
      auto frontLeftState = frc::SwerveModuleState{
          sensor_conversions::swerve_drive::drive::ToVelocity(frontLeftDriveVelocityUpdate.GetValue()),
          frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(frontLeftTurnPosition)}};
      auto frontRightModule = frc::SwerveModulePosition{
          sensor_conversions::swerve_drive::drive::ToDistance(frontRightDrivePosition),
          frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(frontRightTurnPosition)}};
      auto frontRightState = frc::SwerveModuleState{
          sensor_conversions::swerve_drive::drive::ToVelocity(frontRightDriveVelocityUpdate.GetValue()),
          frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(frontRightTurnPosition)}};
      auto backRightModule = frc::SwerveModulePosition{
          sensor_conversions::swerve_drive::drive::ToDistance(backRightDrivePosition),
          frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(backRightTurnPosition)}};
      auto backRightState = frc::SwerveModuleState{
          sensor_conversions::swerve_drive::drive::ToVelocity(backRightDriveVelocityUpdate.GetValue()),
          frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(backRightTurnPosition)}};
      auto backLeftModule = frc::SwerveModulePosition{
          sensor_conversions::swerve_drive::drive::ToDistance(backLeftDrivePosition),
          frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(backLeftTurnPosition)}};
      auto backLeftState = frc::SwerveModuleState{
          sensor_conversions::swerve_drive::drive::ToVelocity(backLeftDriveVelocityUpdate.GetValue()),
          frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(backLeftTurnPosition)}};

      frc::Pose2d poseEstimate;
      {
        std::lock_guard lock{m_poseEstimatorLock};
        poseEstimate = m_poseEstimator.UpdateWithTime(
            updateTime, frc::Rotation2d(yaw), {frontLeftModule, frontRightModule, backRightModule, backLeftModule});
      }
      auto now = std::chrono::steady_clock::now();
      if (now - logTime >= std::chrono::milliseconds{50}) {
        m_poseEstimateLogger.Append(poseEstimate);

        m_stateLogger.Append(std::span<const frc::SwerveModuleState>(std::array<const frc::SwerveModuleState, 4>{
            frontLeftState, frontRightState, backRightState, backLeftState}));
        logTime = now;
      }
    }
  }
}

units::degree_t SwerveDriveSubsystem::GetFieldCentricAngle() {
  return -GetIMUYaw() - m_fieldHomeOffset;
}

frc::Pose2d SwerveDriveSubsystem::GetPoseEstimate(const frc::Pose2d& robotPose, const units::millisecond_t& latency) {
  // Account for Vision Measurement here
  frc::Timer timer;
  const auto timeStamp = timer.GetFPGATimestamp() - latency;
  std::lock_guard lock{m_poseEstimatorLock};
  m_poseEstimator.AddVisionMeasurement(robotPose, timeStamp);

  if constexpr (feature_flags::nt_debugging) {
    frc::SmartDashboard::PutNumber("(Est Pose After Vision) X",
                                   units::inch_t{m_poseEstimator.GetEstimatedPosition().X()}.to<double>());
    frc::SmartDashboard::PutNumber("(Est Pose After Vision) Y",
                                   units::inch_t{m_poseEstimator.GetEstimatedPosition().Y()}.to<double>());
    frc::SmartDashboard::PutNumber("(Est Pose After Vision) Angle",
                                   m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().to<double>());
    frc::SmartDashboard::PutNumber("(Est Pose After Vision) Continuous Angle",
                                   GetContinuousOdometryAngle().Degrees().to<double>());
  }

  return frc::Pose2d{m_poseEstimator.GetEstimatedPosition().Translation(), GetContinuousOdometryAngle()};
}

void SwerveDriveSubsystem::UpdateVisionMeasurement(const frc::Pose2d& poseEstimate,
                                                   units::second_t timestamp,
                                                   const wpi::array<double, 3>& visionMeasurementStdDevs) {
  // Block vision odometry updates right after a position reset since this can cause undesired jumps in robot
  // position estimate while megatag2 position readjusts
  if (std::chrono::steady_clock::now() - m_odometryResetTime > std::chrono::milliseconds{250}) {
    std::lock_guard lock{m_poseEstimatorLock};
    m_poseEstimator.AddVisionMeasurement(poseEstimate, timestamp, visionMeasurementStdDevs);
  }
}

void SwerveDriveSubsystem::SetControlMode(SwerveDriveSubsystem::DriveControlMode controlMode) {
  m_controlMode = controlMode;
}

void SwerveDriveSubsystem::InitializeMotors() {
  // InitializeMotorsFromFS();
}

void SwerveDriveSubsystem::HomeToFS(const units::degree_t& angle) {
  const argos_lib::swerve::SwerveModulePositions homes{
      ConstrainAngle(m_frontLeft.m_encoder.GetAbsolutePosition().GetValue() - angle, 0_deg, 360_deg),
      ConstrainAngle(m_frontRight.m_encoder.GetAbsolutePosition().GetValue() - angle, 0_deg, 360_deg),
      ConstrainAngle(m_backRight.m_encoder.GetAbsolutePosition().GetValue() - angle, 0_deg, 360_deg),
      ConstrainAngle(m_backLeft.m_encoder.GetAbsolutePosition().GetValue() - angle, 0_deg, 360_deg)};

  m_fsStorage.Save(homes);
}

void SwerveDriveSubsystem::InitializeMotorsFromFS() {
  std::optional<argos_lib::swerve::SwerveModulePositions> homes = m_fsStorage.Load();

  if (!homes) {
    // ALERT HERE THAT THERE ARE NO VALUES, BUT FOR NOW, JUST PRINT
    std::printf("%d HEY NO SAVED VALUES IN std::FILE SYSTEM!!!!", __LINE__);
    return;
  }

  // GET CURRENT VALUES
  units::degree_t frontLeft_current = m_frontLeft.m_encoder.GetAbsolutePosition().GetValue();
  units::degree_t frontRight_current = m_frontRight.m_encoder.GetAbsolutePosition().GetValue();
  units::degree_t backRight_current = m_backRight.m_encoder.GetAbsolutePosition().GetValue();
  units::degree_t backLeft_current = m_backLeft.m_encoder.GetAbsolutePosition().GetValue();

  // SUBTRACT SAVED FROM CURRENT
  const units::degree_t frontLeftCalibrated = frontLeft_current - homes.value().FrontLeft;
  const units::degree_t frontRightCalibrated = frontRight_current - homes.value().FrontRight;
  const units::degree_t backRightCalibrated = backRight_current - homes.value().RearRight;
  const units::degree_t backLeftCalibrated = backLeft_current - homes.value().RearLeft;

  // ASSIGN DIFFERENCE TO CURRENT MOTOR RELATIVE POSITION
  m_frontLeft.m_encoder.SetPosition(frontLeftCalibrated, 50_ms);
  m_frontRight.m_encoder.SetPosition(frontRightCalibrated, 50_ms);
  m_backRight.m_encoder.SetPosition(backRightCalibrated, 50_ms);
  m_backLeft.m_encoder.SetPosition(backLeftCalibrated, 50_ms);
}

// SWERVE MODULE SUBSYSTEM FUNCTIONS
SwerveModule::SwerveModule(const argos_lib::CANAddress& driveAddr,
                           const argos_lib::CANAddress& turnAddr,
                           const argos_lib::CANAddress& encoderAddr)
    : m_drive(driveAddr.address, std::string(driveAddr.busName))
    , m_turn(turnAddr.address, std::string(turnAddr.busName))
    , m_encoder(encoderAddr.address, std::string(encoderAddr.busName)) {}

frc::SwerveModuleState SwerveModule::GetState() {
  return frc::SwerveModuleState{
      sensor_conversions::swerve_drive::drive::ToVelocity(m_drive.GetVelocity().GetValue()),
      frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(m_turn.GetPosition().GetValue())}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return frc::SwerveModulePosition{
      sensor_conversions::swerve_drive::drive::ToDistance(m_drive.GetPosition().GetValue()),
      frc::Rotation2d{sensor_conversions::swerve_drive::turn::ToAngle(m_turn.GetPosition().GetValue())}};
}

void SwerveDriveSubsystem::UpdateFollowerLinearPIDParams(double kP, double kI, double kD) {
  m_followerController.GetXController().SetPID(kP, kI, kD);
  m_followerController.GetYController().SetPID(kP, kI, kD);
  m_followerController.GetXController().Reset();
  m_followerController.GetYController().Reset();
}

void SwerveDriveSubsystem::UpdateFollowerRotationalPIDParams(double kP, double kI, double kD) {
  m_followerController.GetThetaController().SetPID(kP, kI, kD);
  m_followerController.GetThetaController().Reset(m_followerController.GetThetaController().GetGoal());
}

void SwerveDriveSubsystem::UpdateFollowerRotationalPIDConstraints(
    frc::TrapezoidProfile<units::radians>::Constraints constraints) {
  m_followerController.GetThetaController().SetConstraints(constraints);
  m_rotationalPIDConstraints = constraints;
  m_followerController.GetThetaController().Reset(m_followerController.GetThetaController().GetGoal());
}

void SwerveDriveSubsystem::UpdateFollowerRotationalPIDConstraints(
    frc::TrapezoidProfile<units::degrees>::Constraints constraints) {
  UpdateFollowerRotationalPIDConstraints(
      frc::TrapezoidProfile<units::radians>::Constraints{constraints.maxVelocity, constraints.maxAcceleration});
}

void SwerveDriveSubsystem::CancelDrivingProfile() {
  m_profileComplete = false;
  m_followingProfile = false;
}

bool SwerveDriveSubsystem::ProfileIsComplete() const {
  return m_profileComplete;
}

bool SwerveDriveSubsystem::IsFollowingProfile() const {
  return m_followingProfile;
}

units::degree_t SwerveDriveSubsystem::GetIMUYaw() {
  return -m_pigeonIMU.GetYaw().GetValue();
}

units::degrees_per_second_t SwerveDriveSubsystem::GetIMUYawRate() {
  return -units::degrees_per_second_t{m_pigeonIMU.GetRate()};
}

void SwerveDriveSubsystem::ResetIMUYaw() {
  m_pigeonIMU.SetYaw(0_deg);
}

frc::ChassisSpeeds SwerveDriveSubsystem::GetChassisVelocity() {
  return m_swerveDriveKinematics.ToChassisSpeeds(GetCurrentModuleStates());
}

wpi::array<frc::SwerveModuleState, 4> SwerveDriveSubsystem::OptimizeAllModules(
    wpi::array<frc::SwerveModuleState, 4> rawStates) {
  rawStates.at(0) = argos_lib::swerve::Optimize(
      rawStates.at(0),
      sensor_conversions::swerve_drive::turn::ToAngle(m_frontLeft.m_turn.GetPosition().GetValue()),
      0_rpm,
      0_fps,
      12_fps);
  rawStates.at(1) = argos_lib::swerve::Optimize(
      rawStates.at(1),
      sensor_conversions::swerve_drive::turn::ToAngle(m_frontRight.m_turn.GetPosition().GetValue()),
      0_rpm,
      0_fps,
      12_fps);
  rawStates.at(2) = argos_lib::swerve::Optimize(
      rawStates.at(2),
      sensor_conversions::swerve_drive::turn::ToAngle(m_backRight.m_turn.GetPosition().GetValue()),
      0_rpm,
      0_fps,
      12_fps);
  rawStates.at(3) = argos_lib::swerve::Optimize(
      rawStates.at(3),
      sensor_conversions::swerve_drive::turn::ToAngle(m_backLeft.m_turn.GetPosition().GetValue()),
      0_rpm,
      0_fps,
      12_fps);
  return rawStates;
}

void SwerveDriveSubsystem::ClosedLoopDrive(wpi::array<frc::SwerveModuleState, 4> moduleStates) {
  // FRONT LEFT
  m_frontLeft.m_drive.SetControl(VelocityVoltage(sensor_conversions::swerve_drive::drive::ToSensorVelocity(
      moduleStates.at(indexes::swerveModules::frontLeftIndex).speed)));

  m_frontLeft.m_turn.SetControl(PositionVoltage(sensor_conversions::swerve_drive::turn::ToSensorUnit(
      moduleStates.at(indexes::swerveModules::frontLeftIndex).angle.Degrees())));

  // FRONT RIGHT
  m_frontRight.m_drive.SetControl(VelocityVoltage(sensor_conversions::swerve_drive::drive::ToSensorVelocity(
      moduleStates.at(indexes::swerveModules::frontRightIndex).speed)));

  m_frontRight.m_turn.SetControl(PositionVoltage(sensor_conversions::swerve_drive::turn::ToSensorUnit(
      moduleStates.at(indexes::swerveModules::frontRightIndex).angle.Degrees())));

  // BACK RIGHT
  m_backRight.m_drive.SetControl(VelocityVoltage(sensor_conversions::swerve_drive::drive::ToSensorVelocity(
      moduleStates.at(indexes::swerveModules::backRightIndex).speed)));

  m_backRight.m_turn.SetControl(PositionVoltage(sensor_conversions::swerve_drive::turn::ToSensorUnit(
      moduleStates.at(indexes::swerveModules::backRightIndex).angle.Degrees())));

  // BACK LEFT
  m_backLeft.m_drive.SetControl(VelocityVoltage(sensor_conversions::swerve_drive::drive::ToSensorVelocity(
      moduleStates.at(indexes::swerveModules::backLeftIndex).speed)));

  m_backLeft.m_turn.SetControl(PositionVoltage(sensor_conversions::swerve_drive::turn::ToSensorUnit(
      moduleStates.at(indexes::swerveModules::backLeftIndex).angle.Degrees())));

  m_setpointLogger.Append(std::span<const frc::SwerveModuleState>(
      std::array<const frc::SwerveModuleState, 4>{moduleStates.at(indexes::swerveModules::frontLeftIndex),
                                                  moduleStates.at(indexes::swerveModules::frontRightIndex),
                                                  moduleStates.at(indexes::swerveModules::backRightIndex),
                                                  moduleStates.at(indexes::swerveModules::backLeftIndex)}));
}
