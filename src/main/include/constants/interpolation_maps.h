/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <array>

#include "argos_lib/general/interpolation.h"

namespace controllerMap {
  using argos_lib::InterpMapPoint;

  [[maybe_unused]] constexpr std::array driveSpeed{InterpMapPoint{-1.0, -1.0},
                                                   //    InterpMapPoint{-0.75, -0.4},
                                                   InterpMapPoint{-0.15, 0.0},
                                                   InterpMapPoint{0.15, 0.0},
                                                   //    InterpMapPoint{0.75, 0.4},
                                                   InterpMapPoint{1.0, 1.0}};
  [[maybe_unused]] constexpr std::array driveRotSpeed{
      InterpMapPoint{-1.0, -1.0}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 1.0}};
  [[maybe_unused]] constexpr std::array elevatorSpeed{
      InterpMapPoint{-1.0, -0.5}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 0.5}};
  [[maybe_unused]] constexpr std::array elevatorRotateSpeed{
      InterpMapPoint{-1.0, -0.2}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 0.2}};
  [[maybe_unused]] constexpr std::array climberSpeed{
      InterpMapPoint{-1.0, -0.2}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 0.2}};
}  // namespace controllerMap
