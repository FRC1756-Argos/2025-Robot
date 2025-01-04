/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/simple_led_subsystem.h"

#include <ctre/phoenix/led/FireAnimation.h>
#include <ctre/phoenix/led/LarsonAnimation.h>
#include <ctre/phoenix/led/SingleFadeAnimation.h>
#include <ctre/phoenix/led/StrobeAnimation.h>
#include <frc/DriverStation.h>

#include <chrono>
#include <numbers>

#include "argos_lib/config/config_types.h"
#include "constants/addresses.h"

using namespace std::chrono_literals;
using ctre::phoenix::led::CANdle;

SimpleLedSubsystem::SimpleLedSubsystem(argos_lib::RobotInstance instance)
    : m_CANdle{instance == argos_lib::RobotInstance::Competition ?
                   std::make_optional<CANdle>(
                       GetCANAddr(address::comp_bot::led::CANdle, address::practice_bot::led::CANdle, instance),
                       std::string(
                           GetCANBus(address::comp_bot::led::CANdle, address::practice_bot::led::CANdle, instance))) :
                   std::nullopt}
    , m_log{"SIMPLE_LED_SUBSYSTEM"}
    , m_enabled{false}
    , m_hasBeenConnected{false}
    , m_disableUpdateFunction{[]() {}}
    , m_ledUpdateFunction{[]() {}}
    , m_restoreAnimationFunction{std::nullopt}
    , m_startTime{std::chrono::steady_clock::now()}
    , m_temporaryDuration{0_ms} {
  SetAllGroupsOff();
}

void SimpleLedSubsystem::Enable() {
  m_enabled = true;
}
void SimpleLedSubsystem::Disable() {
  m_enabled = false;
  if (m_restoreAnimationFunction) {
    m_ledUpdateFunction = m_restoreAnimationFunction.value();
    m_restoreAnimationFunction = std::nullopt;
  }
}

void SimpleLedSubsystem::SetLedsConnectedBrightness(bool connected) {
  if (!m_CANdle) {
    return;  // Do nothing, because there is no CANdle on here anyway
  }
  m_CANdle.value().ConfigBrightnessScalar(connected ? 0.8 : 0.1);
  if (connected) {
    m_hasBeenConnected = true;
  }
}

void SimpleLedSubsystem::SetDisableAnimation(std::function<void()> animationFunction) {
  m_disableUpdateFunction = animationFunction;
}

// This method will be called once per scheduler run
void SimpleLedSubsystem::Periodic() {
  if (m_enabled) {
    m_ledUpdateFunction();
    if (m_restoreAnimationFunction &&
        units::millisecond_t(std::chrono::steady_clock::now() - m_startTime) > m_temporaryDuration) {
      m_ledUpdateFunction = m_restoreAnimationFunction.value();
      m_restoreAnimationFunction = std::nullopt;
    }
  } else {
    m_disableUpdateFunction();
  }
}

void SimpleLedSubsystem::SetLedGroupColor(LedGroup group, argos_lib::ArgosColor color, bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this, group, color]() { this->SetLedGroupColor(group, color, false); };
  }

  int startIndx = -1;
  int len = -1;
  switch (group) {
    case LedGroup::BACK:
      m_CANdle.value().ClearAnimation(0);
      m_CANdle.value().ClearAnimation(1);
      startIndx = startIndex_backLeft;
      len = length_backLeft + length_backRight;
      break;
    case LedGroup::FRONT:
      m_CANdle.value().ClearAnimation(2);
      startIndx = startIndex_front;
      len = length_front;
      break;

    default:
      break;
  }

  if (startIndx < 0 || len < 0) {
    m_log.Log(argos_lib::LogLevel::ERR, "INVALID LED LENGTH OR START INDEX\n");
  }

  ctre::phoenix::ErrorCode rslt;
  rslt = m_CANdle.value().SetLEDs(color.r, color.g, color.b, 0, startIndx, len);
  if (rslt != ctre::phoenix::ErrorCode::OKAY) {
    m_log.Log(argos_lib::LogLevel::ERR, "CANDle::SetLEDs() returned error[%d]", rslt);
  }
}

void SimpleLedSubsystem::SetLedStripColor(LedStrip strip, argos_lib::ArgosColor color, bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this, strip, color]() { this->SetLedStripColor(strip, color, false); };
  }

  int startIndex = -1;
  int len = -1;
  switch (strip) {
    case LedStrip::BackLeft:
      m_CANdle.value().ClearAnimation(0);
      startIndex = startIndex_backLeft;
      len = length_backLeft;
      break;
    case LedStrip::BackRight:
      m_CANdle.value().ClearAnimation(1);
      startIndex = startIndex_backRight;
      len = length_backRight;
      break;
    case LedStrip::Front:
      m_CANdle.value().ClearAnimation(2);
      startIndex = startIndex_front;
      len = length_front;
      break;
  }

  if (startIndex < 0 || len < 0) {
    m_log.Log(argos_lib::LogLevel::ERR, "INVALID LED LENGTH OR START INDEX\n");
  }

  ctre::phoenix::ErrorCode rslt;
  rslt = m_CANdle.value().SetLEDs(color.r, color.g, color.b, 0, startIndex, len);
  if (rslt != ctre::phoenix::ErrorCode::OKAY) {
    m_log.Log(argos_lib::LogLevel::ERR, "CANDle::SetLEDs() returned error[%d]", rslt);
  }
}

void SimpleLedSubsystem::SetAllGroupsColor(argos_lib::ArgosColor color, bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this, color]() { this->SetAllGroupsColor(color, false); };
  }

  StopAllAnimations(false);
  int len = length_backLeft + length_backRight + length_front;
  ctre::phoenix::ErrorCode rslt;
  rslt = m_CANdle.value().SetLEDs(color.r, color.g, color.b, 0, startIndex_backLeft, len);
  if (rslt != ctre::phoenix::ErrorCode::OKAY) {
    m_log.Log(argos_lib::LogLevel::ERR, "CANDle::SetLEDs() returned error[%d]", rslt);
  }
}

void SimpleLedSubsystem::SetAllGroupsFade(argos_lib::ArgosColor color, bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this, color]() { this->SetAllGroupsFade(color, false); };
  }

  const int tipSize = 0;

  std::array<int, 3> lengths = {length_backLeft - tipSize, length_backRight - tipSize, length_front - tipSize};
  std::array<int, 3> offsets = {(inverted_backLeft ? tipSize : 0) + startIndex_backLeft,
                                (inverted_backRight ? tipSize : 0) + startIndex_backRight,
                                (inverted_front ? tipSize : 0) + startIndex_front};
  for (size_t i = 0; i < lengths.size(); ++i) {
    auto fadeAnimation =
        ctre::phoenix::led::SingleFadeAnimation(color.r, color.g, color.b, 0, 0.7, lengths.at(i), offsets.at(i));
    m_CANdle.value().Animate(fadeAnimation, i);
  }
}

void SimpleLedSubsystem::SetAllGroupsFlash(argos_lib::ArgosColor color, bool restorable) {
  if (!m_CANdle) {
    return;
  }
  if (restorable) {
    m_ledUpdateFunction = [this, color]() { this->SetAllGroupsFlash(color, false); };
  }

  std::array<int, 3> lengths = {length_backLeft, length_backRight, length_front};
  std::array<int, 3> offsets = {startIndex_backLeft, startIndex_backRight, startIndex_front};

  for (size_t i = 0; i < lengths.size(); ++i) {
    auto flashAnimation =
        ctre::phoenix::led::StrobeAnimation(color.r, color.g, color.b, 0, 0.1, lengths.at(i), offsets.at(i));
    m_CANdle.value().Animate(flashAnimation, i);
  }
}

void SimpleLedSubsystem::FlashStrip(LedStrip strip, argos_lib::ArgosColor color, bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing;
  }

  if (restorable) {
    m_ledUpdateFunction = [this, color]() { this->SetAllGroupsFlash(color, false); };
  }

  int startIndex = -1;
  int len = -1;

  switch (strip) {
    case LedStrip::BackLeft:
      startIndex = startIndex_backLeft;
      len = length_backLeft;
      break;
    case LedStrip::BackRight:
      startIndex = startIndex_backRight;
      len = length_backRight;
      break;
    case LedStrip::Front:
      startIndex = startIndex_front;
      len = length_front;
      break;
  }

  if (startIndex < 0 || len < 0) {
    m_log.Log(argos_lib::LogLevel::ERR, "INVALID LED LENGTH OR START INDEX\n");
  }

  auto flashAnimation = ctre::phoenix::led::StrobeAnimation(color.r, color.g, color.b, 0, 0.1, len, startIndex);
  auto rslt = m_CANdle.value().Animate(flashAnimation, static_cast<int>(strip));

  if (rslt != ctre::phoenix::ErrorCode::OKAY) {
    m_log.Log(argos_lib::LogLevel::ERR, "CANDle::SetLEDs() returned error[%d]", rslt);
  }
}

void SimpleLedSubsystem::SetAllGroupsLarson(argos_lib::ArgosColor color, bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this, color]() { this->SetAllGroupsLarson(color, false); };
  }

  std::array<int, 3> lengths = {length_backLeft, length_backRight, length_front};
  std::array<int, 3> offsets = {startIndex_backLeft, startIndex_backRight, startIndex_front};

  for (size_t i = 0; i < lengths.size(); ++i) {
    auto larsonAnimation = ctre::phoenix::led::LarsonAnimation(color.r,
                                                               color.g,
                                                               color.b,
                                                               0,
                                                               0.1,
                                                               lengths.at(i),
                                                               ctre::phoenix::led::LarsonAnimation::Front,
                                                               10,
                                                               offsets.at(i));
    m_CANdle.value().Animate(larsonAnimation, i);
  }
}

argos_lib::ArgosColor SimpleLedSubsystem::GetAllianceColor() {
  if (!m_hasBeenConnected) {
    return argos_lib::gamma_corrected_colors::kCatYellow;
  }
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance) {
    m_latestAlliance = alliance.value();
  }
  if (m_latestAlliance == frc::DriverStation::Alliance::kBlue) {
    return argos_lib::colors::kReallyBlue;
  } else {
    return argos_lib::colors::kReallyRed;
  }
}

void SimpleLedSubsystem::SetAllGroupsAllianceColor(bool fade, bool restorable) {
  if (restorable) {
    m_ledUpdateFunction = [this, fade]() { this->SetAllGroupsAllianceColor(fade, false); };
  }

  auto color = GetAllianceColor();
  if (fade) {
    SetAllGroupsFade(color, false);
  } else {
    SetAllGroupsColor(color, false);
  }
}

void SimpleLedSubsystem::StopAllAnimations(bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this]() { this->StopAllAnimations(false); };
  }

  m_CANdle.value().ClearAnimation(0);
  m_CANdle.value().ClearAnimation(1);
  m_CANdle.value().ClearAnimation(2);
}

void SimpleLedSubsystem::SetAllGroupsOff(bool restorable) {
  if (restorable) {
    m_ledUpdateFunction = [this]() { this->SetAllGroupsOff(false); };
  }

  SetAllGroupsColor(argos_lib::colors::kOff, false);
}

void SimpleLedSubsystem::FireEverywhere(bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }
  if (restorable) {
    m_ledUpdateFunction = [this]() { this->FireEverywhere(false); };
  }

  std::array<int, 3> lengths = {length_backLeft, length_backRight, length_front};
  std::array<int, 3> offsets = {startIndex_backLeft, startIndex_backRight, startIndex_front};
  std::array<bool, 3> inverts = {inverted_backLeft, inverted_backRight, inverted_front};
  for (size_t i = 0; i < lengths.size(); ++i) {
    auto fireAnimation =
        ctre::phoenix::led::FireAnimation(0.5, 0.15, lengths.at(i), 0.9, 0.3, inverts.at(i), offsets.at(i));
    m_CANdle.value().Animate(fireAnimation, i);
  }
}

void SimpleLedSubsystem::ColorSweep(argos_lib::ArgosColor color, bool correctGamma, bool restorable) {
  // if (restorable) {
  //   m_restoreAnimationFunction = [this, color, correctGamma]() { this->ColorSweep(color, correctGamma, false); };
  // }
  // // No continuous update required
  // m_ledUpdateFunction = [this, color, correctGamma]() { this->ColorSweep(color, correctGamma, false); };

  // const auto period_ms = 2000.0;

  // const auto now = std::chrono::steady_clock::now();
  // const auto timeWithinPeriod =
  //     std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() -
  //     std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) / (1ms * period_ms);

  // const auto frontTime = std::fmod(static_cast<double>(timeWithinPeriod), period_ms);
  // const auto sideFrontTime = std::fmod(frontTime + 250, period_ms);
  // const auto sideBackTime = std::fmod(sideFrontTime + 250, period_ms);
  // const auto backTime = std::fmod(sideBackTime + 250, period_ms);

  // const auto frontAngle = 2 * std::numbers::pi * frontTime / period_ms;
  // const auto sideFrontAngle = 2 * std::numbers::pi * sideFrontTime / period_ms;
  // const auto sideBackAngle = 2 * std::numbers::pi * sideBackTime / period_ms;
  // const auto backAngle = 2 * std::numbers::pi * backTime / period_ms;

  // auto frontColor = color * (0.5 + std::sin(frontAngle) / 2.0);
  // auto sideFrontColor = color * (0.5 + std::sin(sideFrontAngle) / 2.0);
  // auto sideBackColor = color * (0.5 + std::sin(sideBackAngle) / 2.0);
  // auto backColor = color * (0.5 + std::sin(backAngle) / 2.0);

  // if (correctGamma) {
  //   frontColor = argos_lib::GammaCorrect(frontColor);
  //   sideFrontColor = argos_lib::GammaCorrect(sideFrontColor);
  //   sideBackColor = argos_lib::GammaCorrect(sideBackColor);
  //   backColor = argos_lib::GammaCorrect(backColor);
  // }

  // SetLedStripColor(LedStrip::FrontLeft, frontColor, false);
  // SetLedStripColor(LedStrip::FrontRight, frontColor, false);
  // SetLedStripColor(LedStrip::SideFront, sideFrontColor, false);
  // SetLedStripColor(LedStrip::SideBack, sideBackColor, false);
  // SetLedStripColor(LedStrip::BackLeft, backColor, false);
  // SetLedStripColor(LedStrip::BackRight, backColor, false);
}

void SimpleLedSubsystem::TemporaryAnimate(std::function<void()> animationFunction, units::millisecond_t duration) {
  m_startTime = std::chrono::steady_clock::now();
  m_temporaryDuration = duration;
  if (!m_restoreAnimationFunction) {
    m_restoreAnimationFunction = m_ledUpdateFunction;
  }
  m_ledUpdateFunction = animationFunction;
}
