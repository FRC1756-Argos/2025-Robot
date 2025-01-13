/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/subsystems/led_subsystem.h"

#include <algorithm>

using namespace argos_lib;

bool operator==(const frc::AddressableLED::LEDData& a, const frc::AddressableLED::LEDData& b) {
  return a.r == b.r && a.g == b.g && a.b == b.b;
}

bool operator!=(const frc::AddressableLED::LEDData& a, const frc::AddressableLED::LEDData& b) {
  return !(a == b);
}

bool LEDState::operator==(const LEDState& other) const {
  return animated == other.animated && color == other.color;
}
bool LEDState::operator!=(const LEDState& other) const {
  return !operator==(other);
}

LEDState& LEDState::operator=(const frc::AddressableLED::LEDData& other) {
  color = other;
  return *this;
}

LEDSubsystem::LEDSubsystem(unsigned numAuxLEDs)
    : m_currentLEDs{numAuxLEDs + numIntegratedLEDs}
    , m_prevLEDs{numAuxLEDs + numIntegratedLEDs}
    , m_controller{1, "rio"} {}

void LEDSubsystem::Periodic() {
  // Use animation functions to determine all new LED states
  for (auto& animation : m_customAnimations) {
    const auto animatedString = animation.update();
    std::copy_n(animatedString.begin(), animation.numLEDs, std::next(m_currentLEDs.begin(), animation.offset));
  }
  // Only send updates for LEDs that changed to reduce CAN messages
  const auto updatesToSend = GetDeltaUpdate(m_prevLEDs, m_currentLEDs);
  for (const auto& update : updatesToSend) {
    m_controller.SetLEDs(update.color.r, update.color.g, update.color.b, 0, update.startIndex, update.numLEDs);
  }
  m_prevLEDs = m_currentLEDs;
}

void LEDSubsystem::StockAnimateAuxLEDs(ctre::phoenix::led::Animation& animation, int slot) {
  animation.SetLedOffset(animation.GetLedOffset() + numIntegratedLEDs);
  std::fill_n(std::next(m_currentLEDs.begin(), animation.GetLedOffset()), animation.GetNumLed(), LEDState{true, {}});
  m_controller.Animate(animation, slot);
}

void LEDSubsystem::StockAnimateIntegratedLEDs(ctre::phoenix::led::Animation& animation, int slot) {
  // Ensure we stay within the number of LEDs in this subsystem
  std::fill_n(m_currentLEDs.begin(), numIntegratedLEDs, LEDState{true, {}});
  animation.SetNumLed(numIntegratedLEDs);
  animation.SetLedOffset(0);
  m_controller.Animate(animation, slot);
}

void LEDSubsystem::CustomAnimateAuxLEDs(argos_lib::led::Animation animation) {
  // Ensure we stay within the number of LEDs in this subsystem
  animation.offset += numIntegratedLEDs;
  animation.numLEDs = std::min<size_t>(std::max<size_t>(0, m_currentLEDs.size() - animation.offset), animation.numLEDs);
  std::fill_n(std::next(m_currentLEDs.begin(), animation.offset), animation.numLEDs, LEDState{false, {}});
  m_customAnimations.push_back(animation);
}

void LEDSubsystem::CustomAnimateIntegratedLEDs(argos_lib::led::Animation animation) {
  // Ensure this animation doesn't go past the integrated LED addresses
  if (animation.numLEDs > numIntegratedLEDs) {
    animation.numLEDs = numIntegratedLEDs;
  }
  animation.offset = 0;
  std::fill_n(m_currentLEDs.begin(), animation.numLEDs, LEDState{false, {}});
  m_customAnimations.push_back(animation);
}

std::vector<LEDSubsystem::LEDUpdateGroup> LEDSubsystem::GetDeltaUpdate(const std::vector<LEDState>& prev,
                                                                       const std::vector<LEDState>& current) {
  std::vector<bool> changed(current.size());
  // Only update LEDs that changed and are not using built-in animations
  std::transform(
      prev.begin(), prev.end(), current.begin(), changed.begin(), [](const LEDState& prev, const LEDState& current) {
        return current.animated == false && current != prev;
      });

  // Find blocks of identical colors
  auto searchStart = current.begin();
  std::vector<LEDSubsystem::LEDUpdateGroup> retVal;
  while (searchStart != current.end()) {
    auto rangeEnd =
        std::adjacent_find(searchStart, current.end(), [](const LEDState& a, const LEDState& b) { return a != b; });
    if (rangeEnd == current.end()) {
      rangeEnd = std::prev(rangeEnd);
    }
    unsigned numLEDs = std::distance(searchStart, rangeEnd) + 1;
    unsigned offset = std::distance(current.begin(), searchStart);
    // Skip updates for sections that we shouldn't be updating
    if (std::any_of(std::next(changed.begin(), offset), std::next(changed.begin(), offset + numLEDs), [](bool changed) {
          return changed;
        })) {
      retVal.emplace_back(offset, numLEDs, searchStart->color);
    }
    searchStart = std::next(rangeEnd);
  }
  return retVal;
}
