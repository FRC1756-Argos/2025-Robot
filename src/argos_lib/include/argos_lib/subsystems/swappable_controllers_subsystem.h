/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "argos_lib/controller/xbox_controller.h"

namespace argos_lib {

  /**
 * @brief Allows two controllers to swap between "Driver" and "Operator" control schemes on the fly
 * by swapping the m_driverController and m_operatorController objects
 *
 */
  class SwappableControllersSubsystem : public frc2::SubsystemBase {
   public:
    SwappableControllersSubsystem() = delete;
    /**
     * @brief Construct a new Swappable Controllers Subsystem object with two controllers
     *
     * @param driverControllerPort Index of driver controller
     * @param operatorControllerPort Index of operator controller
     */
    SwappableControllersSubsystem(int driverControllerPort, int operatorControllerPort);

    /**
     * @brief Swap driver and operator controller rolls
     */
    void Swap();

    /**
     * @brief Get reference to active driver controller
     *
     * @return Active driver controller based on swaps up to this point
     */
    argos_lib::XboxController& DriverController();
    /**
     * @brief Get reference to active operator controller
     *
     * @return Active operator controller based on swaps up to this point
     */
    argos_lib::XboxController& OperatorController();

    /**
     * @brief Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * @brief Change vibration model for both controllers
     */
    void VibrateAll(argos_lib::VibrationModel newModel);

   private:
    argos_lib::XboxController m_driverController;    ///< Managed driver controller
    argos_lib::XboxController m_operatorController;  ///< Managed operator controller
    bool m_swapped;                                  ///< Indicates if controllers are currently swapped

    /**
     * @brief Update vibration on both controllers based on their active vibration models
     */
    void UpdateVibration();
  };

}  // namespace argos_lib
