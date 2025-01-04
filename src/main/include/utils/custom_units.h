/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/base.h>
#include <units/length.h>
#include <units/time.h>

namespace units {
  UNIT_ADD(velocity,
           inches_per_second,
           inches_per_second,
           ips,
           units::compound_unit<units::length::inches, units::inverse<units::time::second>>);
  UNIT_ADD(acceleration,
           inches_per_second_squared,
           inches_per_second_squared,
           ips2,
           units::compound_unit<units::length::inches, units::inverse<units::squared<units::time::second>>>);
}  // namespace units
