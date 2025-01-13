/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "argos_lib/subsystems/swappable_controllers_subsystem.h"

namespace argos_lib {

  /**
   * @brief Swaps rolls of two controllers.  Useful if one controller breaks during a match
   */
  class SwapControllersCommand : public frc2::CommandHelper<frc2::Command, SwapControllersCommand> {
   public:
    explicit SwapControllersCommand(SwappableControllersSubsystem* controllers);

    /**
     * @brief Indicate swap has started, but don't actually swap yet
     */
    void Initialize() override;

    /**
     * @brief Swap controllers when trigger ends
     */
    void End(bool) override;

   private:
    SwappableControllersSubsystem* m_pControllerSubsystem;
  };

}  // namespace argos_lib
