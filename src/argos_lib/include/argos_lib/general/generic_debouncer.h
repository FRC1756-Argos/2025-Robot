/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/time.h>

#include <chrono>

namespace argos_lib {
  template <class T>
  class GenericDebouncer {
   public:
    /**
   * @brief Construct a new Debouncer object with symmetric times
   *
   * @param symmetricDebounceTime Symmetric activate and clear time
   */
    explicit GenericDebouncer(const units::millisecond_t symmetricDebounceTime, T initialValue = {})
        : m_debounceTime{symmetricDebounceTime}
        , m_rawStatus{initialValue}
        , m_debouncedStatus{initialValue}
        , m_debounceTransitionTime{std::chrono::steady_clock::now()} {}

    /**
   * @brief Update debouncer with new input and retreive latest debounced status
   *
   * @param newVal Latest raw value
   * @return Value after applying debounce
   */
    [[nodiscard]] T operator()(T newVal) {
      const auto prevRawValue = m_rawStatus;
      const auto curTime = std::chrono::steady_clock::now();

      if (newVal != m_debouncedStatus && newVal != prevRawValue) {
        m_debounceTransitionTime = curTime;
      }

      if (newVal != m_debouncedStatus) {
        const auto timeSinceTransition = units::millisecond_t{static_cast<double>(
            std::chrono::duration_cast<std::chrono::milliseconds>(curTime - m_debounceTransitionTime).count())};
        if (timeSinceTransition >= m_debounceTime) {
          m_debouncedStatus = newVal;
        }
      }

      m_rawStatus = newVal;
      return m_debouncedStatus;
    }

    /**
   *
   * @param newVal Value to set to
   */
    void Reset(T newVal) {
      m_rawStatus = newVal;
      m_debouncedStatus = newVal;
    }

   private:
    units::millisecond_t m_debounceTime;
    T m_rawStatus;        ///< Last raw value
    T m_debouncedStatus;  ///< Current value after applying debounce
    std::chrono::time_point<std::chrono::steady_clock>
        m_debounceTransitionTime;  ///< Time when latest transition detected
  };

}  // namespace argos_lib
