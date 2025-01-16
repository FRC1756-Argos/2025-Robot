/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <fmt/format.h>
#include <gtest/gtest.h>
#include <units/math.h>

namespace argos_lib {
  namespace testing {

    template <class UnitTypeLhs, class UnitTypeRhs, class UnitTypeTol>
    ::testing::AssertionResult UnitEqual(UnitTypeLhs actual, UnitTypeRhs expected, UnitTypeTol tolerance) {
      if (units::math::abs(actual - expected) < tolerance) {
        return ::testing::AssertionSuccess();
      }
      return ::testing::AssertionFailure()
             << fmt::format("{:.4f} not within {:.4f} of expected {:.4f}", actual, tolerance, expected);
    }

    template <class UnitTypeLhs, class UnitTypeRhs>
    ::testing::AssertionResult UnitEqual(UnitTypeLhs actual, UnitTypeRhs expected) {
      return UnitEqual(actual, expected, decltype(actual)(0.001));
    }

  }  // namespace testing
}  // namespace argos_lib
