/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/general/debouncer.h"

using argos_lib::Debouncer;

Debouncer::Debouncer(const argos_lib::DebounceSettings debounceSettings, bool initialValue)
    : m_debounceSettings{debounceSettings}
    , m_rawStatus{initialValue}
    , m_debouncedStatus{initialValue}
    , m_debounceTransitionTime{std::chrono::steady_clock::now()} {}

Debouncer::Debouncer(const units::millisecond_t symmetricDebounceTime)
    : Debouncer{argos_lib::DebounceSettings{symmetricDebounceTime, symmetricDebounceTime}} {}

[[nodiscard]] bool Debouncer::operator()(bool newVal) {
  const auto prevRawValue = m_rawStatus;
  const auto curTime = std::chrono::steady_clock::now();

  if (prevRawValue == m_debouncedStatus && newVal != m_debouncedStatus) {
    m_debounceTransitionTime = curTime;
  }

  if (newVal != m_debouncedStatus) {
    const auto timeSinceTransition = units::millisecond_t{static_cast<double>(
        std::chrono::duration_cast<std::chrono::milliseconds>(curTime - m_debounceTransitionTime).count())};
    if (newVal) {
      if (timeSinceTransition >= m_debounceSettings.activateTime) {
        m_debouncedStatus = newVal;
      }
    } else {
      if (timeSinceTransition >= m_debounceSettings.clearTime) {
        m_debouncedStatus = newVal;
      }
    }
  }

  m_rawStatus = newVal;
  return m_debouncedStatus;
}

bool Debouncer::GetRawStatus() {
  return m_rawStatus;
}

bool Debouncer::GetDebouncedStatus() {
  return m_debouncedStatus;
}

void Debouncer::Reset(bool newVal) {
  m_rawStatus = newVal;
  m_debouncedStatus = newVal;
}
