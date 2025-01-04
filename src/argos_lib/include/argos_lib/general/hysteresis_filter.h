/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

namespace argos_lib {

  /**
 * @brief A simple hysteresis filter for giving a bool output for a threshold
 *
 * @tparam T Type of threshold values
 */
  template <typename T>
  class HysteresisFilter {
   public:
    HysteresisFilter() = delete;

    /**
     * @brief Construct a new Hysteresis Filter object
     *
     * @param deactivateThreshold Value below which output becomes false
     * @param activateThreshold Value above which output becomes true
     */
    HysteresisFilter(T deactivateThreshold, T activateThreshold)
        : m_activateThreshold(activateThreshold), m_deactivateThreshold(deactivateThreshold) {}

    /**
     * @brief Gets new status after applying hysteresis
     *
     * @param newValue Latest raw value to update the filter
     * @return Filtered value after applying newValue
     */
    [[nodiscard]] bool operator()(T newValue) {
      if (m_currentState) {
        if (newValue < m_deactivateThreshold) {
          m_currentState = false;
        }
      } else {
        if (newValue > m_activateThreshold) {
          m_currentState = true;
        }
      }
      return m_currentState;
    }

   private:
    const T m_activateThreshold;    ///< Threshold above which output becomes true
    const T m_deactivateThreshold;  ///< Threshold below which output becomes false
    bool m_currentState;            ///< Latest state after applying hysteresis
  };
}  // namespace argos_lib
