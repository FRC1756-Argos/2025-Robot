/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/button/Trigger.h>

#include <vector>

namespace argos_lib {
  namespace triggers {
    /**
     * @brief Generates composite trigger that is true when exactly one of the triggers is true
     *
     * @param triggers Triggers to check
     * @return frc2::Trigger Composite trigger
     */
    frc2::Trigger OneOf(std::vector<frc2::Trigger> triggers);

    /**
     * @brief Generates composite trigger that is true when none of the triggers is true
     *
     * @param triggers Triggers to check
     * @return frc2::Trigger Composite trigger
     */
    frc2::Trigger NoneOf(std::vector<frc2::Trigger> triggers);

    /**
     * @brief Generates composite trigger that is true when any of the triggers is true
     *
     * @param triggers Triggers to check
     * @return frc2::Trigger Composite trigger
     */
    frc2::Trigger AnyOf(std::vector<frc2::Trigger> triggers);

    /**
     * @brief Generates composite trigger that is true when all of the triggers are true
     *
     * @param triggers Triggers to check
     * @return frc2::Trigger Composite trigger
     */
    frc2::Trigger AllOf(std::vector<frc2::Trigger> triggers);
  }  // namespace triggers
}  // namespace argos_lib
