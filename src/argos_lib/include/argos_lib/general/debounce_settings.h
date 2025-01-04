/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/time.h>

namespace argos_lib {

  struct DebounceSettings {
    units::millisecond_t activateTime;
    units::millisecond_t clearTime;
  };

}  // namespace argos_lib
