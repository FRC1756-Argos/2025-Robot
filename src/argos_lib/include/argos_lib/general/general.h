/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

namespace argos_lib {

  /**
   * @brief Detect if a value is within a threshold of a target value
   *
   * @tparam T Type that implements operator+(), operator-(), operator<=() and operator>=()
   * @param value Value to check
   * @param target Center of range
   * @param threshold Allowable error from target
   * @return true when value is within threshold of target, false otherwise
   */
  template <typename T>
  constexpr static bool InThreshold(const T value, const T target, const T threshold) {
    return value >= target - threshold && value <= target + threshold;
  }
}  // namespace argos_lib
