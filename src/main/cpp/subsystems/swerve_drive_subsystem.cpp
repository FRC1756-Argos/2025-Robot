/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/swerve_drive_subsystem.h"

#include <argos_lib/config/cancoder_config.h>
#include <argos_lib/config/falcon_config.h>
#include <argos_lib/general/angle_utils.h>
#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/frequency.h>
#include <units/velocity.h>

#include <format>
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
    , m_xPID(instance == argos_lib::RobotInstance::Competition ?
                 frc::PIDController{controlLoop::comp_bot::drive::linear_follower::kP,
                                    controlLoop::comp_bot::drive::linear_follower::kI,
                                    controlLoop::comp_bot::drive::linear_follower::kD} :
                 frc::PIDController{controlLoop::practice_bot::drive::linear_follower::kP,
                                    controlLoop::practice_bot::drive::linear_follower::kI,
                                    controlLoop::practice_bot::drive::linear_follower::kD})
    , m_yPID(instance == argos_lib::RobotInstance::Competition ?
                 frc::PIDController{controlLoop::comp_bot::drive::linear_follower::kP,
                                    controlLoop::comp_bot::drive::linear_follower::kI,
                                    controlLoop::comp_bot::drive::linear_follower::kD} :
                 frc::PIDController{controlLoop::practice_bot::drive::linear_follower::kP,
                                    controlLoop::practice_bot::drive::linear_follower::kI,
                                    controlLoop::practice_bot::drive::linear_follower::kD})
    , m_thetaPID(instance == argos_lib::RobotInstance::Competition ?
                     frc::PIDController{controlLoop::comp_bot::drive::rotational_follower::kP,
                                        controlLoop::comp_bot::drive::rotational_follower::kI,
                                        controlLoop::comp_bot::drive::rotational_follower::kD} :
                     frc::PIDController{controlLoop::practice_bot::drive::rotational_follower::kP,
                                        controlLoop::practice_bot::drive::rotational_follower::kI,
                                        controlLoop::practice_bot::drive::rotational_follower::kD})
    , m_driveMotorPIDTuner{nullptr}
    , m_linearFollowerTuner_P{nullptr}
    , m_linearFollowerTuner_I{nullptr}
    , m_linearFollowerTuner_D{nullptr}
    , m_rotationalFollowerTuner_P{nullptr}
    , m_rotationalFollowerTuner_I{nullptr}
    , m_rotationalFollowerTuner_D{nullptr}
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

  m_thetaPID.EnableContinuousInput(units::radian_t(-180_deg).value(), units::radian_t(180_deg).value());

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
    m_linearFollowerTuner_P->AddMonitor(
        "kP",
        [this](double val) {
          m_xPID.SetP(val);
          m_yPID.SetP(val);
        },
        m_xPID.GetP());
    m_linearFollowerTuner_I->AddMonitor(
        "kI",
        [this](double val) {
          m_xPID.SetI(val);
          m_yPID.SetI(val);
        },
        m_xPID.GetI());
    m_linearFollowerTuner_D->AddMonitor(
        "kD",
        [this](double val) {
          m_xPID.SetD(val);
          m_yPID.SetD(val);
        },
        m_xPID.GetD());
    m_rotationalFollowerTuner_P->AddMonitor("kP", [this](double val) { m_thetaPID.SetP(val); }, m_thetaPID.GetP());
    m_rotationalFollowerTuner_I->AddMonitor("kI", [this](double val) { m_thetaPID.SetI(val); }, m_thetaPID.GetI());
    m_rotationalFollowerTuner_D->AddMonitor("kD", [this](double val) { m_thetaPID.SetD(val); }, m_thetaPID.GetD());
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
  // take care of motors state based on simulated physics
  SimDrive();

  // for robot field visualization
  m_field.SetRobotPose(GetContinuousOdometry());
}

void SwerveDriveSubsystem::SimDrive() {
  constexpr auto dt = 0.02_s;

  // Compute desired module states based on simulation chassis speeds.
  frc::ChassisSpeeds chassisSpeeds{units::meters_per_second_t(m_simVelocities.fwVelocity),
                                   units::meters_per_second_t(m_simVelocities.sideVelocity),
                                   units::radians_per_second_t(m_simVelocities.rotVelocity)};
  auto desiredStates = m_swerveDriveKinematics.ToSwerveModuleStates(chassisSpeeds);
  desiredStates =
      GetRawModuleStates({m_simVelocities.fwVelocity, m_simVelocities.sideVelocity, m_simVelocities.rotVelocity});
  desiredStates = OptimizeAllModules(desiredStates);

  // Update simulation state for each swerve module.
  m_frontLeft.SimulationPeriodic(desiredStates.at(indexes::swerveModules::frontLeftIndex), dt);
  m_frontRight.SimulationPeriodic(desiredStates.at(indexes::swerveModules::frontRightIndex), dt);
  m_backRight.SimulationPeriodic(desiredStates.at(indexes::swerveModules::backRightIndex), dt);
  m_backLeft.SimulationPeriodic(desiredStates.at(indexes::swerveModules::backLeftIndex), dt);

  // Calculate the change in yaw in radians over the timestep and
  // Set the new yaw in the Pigeon2 simulation state.
  auto& pigeonSimState = m_pigeonIMU.GetSimState();
  double angularVelocityDegPerSec = units::degree_t(m_simVelocities.rotVelocity * (180.0 / 3.14159265358)).value();
  m_simulatedHeading += angularVelocityDegPerSec * dt.value();
  pigeonSimState.SetRawYaw(units::angle::degree_t(m_simulatedHeading));
  pigeonSimState.SetAngularVelocityZ(units::degrees_per_second_t(angularVelocityDegPerSec));
}

void SwerveDriveSubsystem::SetTrajectoryDisplay(const std::vector<frc::Pose2d>& trajectory) {
  // First clear existing trajectory
  frc::Field2d newField;
  newField.SetRobotPose(m_field.GetRobotPose());
  std::swap(newField, m_field);
  // Add new trajectory
  for (size_t i = 0; i < trajectory.size(); i++) {
    std::string name = "Waypoint " + std::to_string(i);
    m_field.GetObject(name)->SetPose(trajectory[i]);
  }
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
    m_simVelocities.fwVelocity = -velocities.fwVelocity;
    m_simVelocities.sideVelocity = -velocities.sideVelocity;
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

  if (frc::RobotBase::IsSimulation()) {
    m_simVelocities.fwVelocity = -desiredChassisSpeed.vx.value();
    m_simVelocities.sideVelocity = -desiredChassisSpeed.vy.value();
    m_simVelocities.rotVelocity = desiredChassisSpeed.omega.value();
  }

  ClosedLoopDrive(moduleStates);
}

void SwerveDriveSubsystem::SwerveDrive(const choreo::SwerveSample& sample) {
  auto pose = GetContinuousOdometry();

  // Calculate feedback velocities
  units::meters_per_second_t xFeedback{m_xPID.Calculate(pose.X().value(), sample.x.value())};
  units::meters_per_second_t yFeedback{m_yPID.Calculate(pose.Y().value(), sample.y.value())};
  units::radians_per_second_t headingFeedback{
      m_thetaPID.Calculate(pose.Rotation().Radians().value(), sample.heading.value())};

  // Generate the next speeds for the robot
  frc::ChassisSpeeds speeds{sample.vx + xFeedback, sample.vy + yFeedback, sample.omega + headingFeedback};

  std::cout << std::format("vx={}m/s vy={}m/s o={}rad/s\n", sample.vx.value(), sample.vy.value(), sample.omega.value());
  std::cout << std::format("\tx={}m y={}m o={}rad\n", sample.x.value(), sample.y.value(), sample.heading.value());

  // Apply the generated speeds
  SwerveDrive(speeds);
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

  if (frc::RobotBase::IsSimulation()) {
    m_simVelocities.fwVelocity = 0;
    m_simVelocities.sideVelocity = 0;
    m_simVelocities.rotVelocity = 0;
  }
}

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

  if (frc::RobotBase::IsSimulation()) {
    m_field.SetRobotPose(currentPose);
    m_simulatedHeading = currentPose.Rotation().Degrees().value();
    m_pigeonIMU.GetSimState().SetRawYaw(currentPose.Rotation().Degrees());
    ResetIMUYaw();
  }

  {
    std::lock_guard lock{m_poseEstimatorLock};
    m_poseEstimator.ResetPosition(-GetIMUYaw(), GetCurrentModulePositions(), currentPose);
    m_prevOdometryAngle = m_poseEstimator.GetEstimatedPosition().Rotation().Degrees();
  }
  m_continuousOdometryOffset = 0_deg;

  m_xPID.Reset();
  m_yPID.Reset();
  m_thetaPID.Reset();

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

void SwerveModule::SimulationPeriodic(const frc::SwerveModuleState& desiredState, units::second_t dt) {
  // Convert desired drive speed (units::velocity::mps) to sensor units
  auto driveSensorVelocity = sensor_conversions::swerve_drive::drive::ToSensorVelocity(desiredState.speed);

  // Integrate drive position over time
  double rawRpm = driveSensorVelocity.value();
  auto driveSensorVelocity_rps = units::turns_per_second_t(rawRpm / 60.0);
  m_simDrivePos += driveSensorVelocity_rps * dt;

  // Update the drive motor's simulation state
  m_drive.GetSimState().SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  m_drive.GetSimState().SetRawRotorPosition(m_simDrivePos);
  m_drive.GetSimState().SetRotorVelocity(units::turns_per_second_t(driveSensorVelocity));

  // For the turning motor, convert the desired angle to sensor units
  auto turnSensorPosition = sensor_conversions::swerve_drive::turn::ToSensorUnit(desiredState.angle.Degrees());

  // Update the turn motor simulation state.
  m_turn.GetSimState().SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  m_turn.GetSimState().SetRawRotorPosition(turnSensorPosition);
  m_turn.GetSimState().SetRotorVelocity(0_tps);  // Zero velocity for simplicity, not sure

  // update the CANcoder simulation state
  m_encoder.GetSimState().SetRawPosition(turnSensorPosition);
}

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
  m_xPID.SetPID(kP, kI, kD);
  m_yPID.SetPID(kP, kI, kD);
  m_xPID.Reset();
  m_yPID.Reset();
}

void SwerveDriveSubsystem::UpdateFollowerRotationalPIDParams(double kP, double kI, double kD) {
  m_thetaPID.SetPID(kP, kI, kD);
  m_thetaPID.Reset();
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
  return m_pigeonIMU.GetAngularVelocityZWorld().GetValue();
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
