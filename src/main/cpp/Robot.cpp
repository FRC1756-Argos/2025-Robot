/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc2/command/CommandScheduler.h>

Robot::Robot() : TimedRobot{}, m_connectedToFieldDebouncer{{0_ms, 30_s}} {
  // Start recording to data log
  frc::DataLogManager::Start();

  // Record DS control and joystick data.
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog(), true);
}

void Robot::RobotInit() {
  m_lastAlliance = frc::DriverStation::GetAlliance();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  // Check to see if alliance has changed
  auto curAlliance = frc::DriverStation::GetAlliance();
  if (curAlliance && (!m_lastAlliance || m_lastAlliance.value() != curAlliance.value())) {
    m_container.AllianceChanged();
    m_lastAlliance = frc::DriverStation::GetAlliance();
  }

  m_container.SetLedsConnectedBrightness(
      m_connectedToFieldDebouncer(frc::DriverStation::IsFMSAttached() || frc::DriverStation::IsDSAttached()));
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  m_container.Disable();
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  // Call robotcontainer enable
  m_container.Enable();

  m_pAutonomousCommand = m_container.GetAutonomousCommand();

  if (m_pAutonomousCommand) {
    m_pAutonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // Call robotcontainer enable
  m_container.Enable();

  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_pAutonomousCommand) {
    m_pAutonomousCommand->Cancel();
    m_pAutonomousCommand = nullptr;
    m_container.AllianceChanged();  // Reset LED pattern on transition from auto to teleop
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
