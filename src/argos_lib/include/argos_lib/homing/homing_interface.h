/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include "argos_lib/general/swerve_utils.h"

namespace argos_lib {
  /**
   * @brief Interface capable of saving and loading module home positions from persistent storage
   */
  class SwerveHomeStorageInterface {
   public:
    /**
     * @brief Save home position to persistent storage
     *
     * @param homePosition Positions to store
     * @return true Save successful
     * @return false Error saving
     */
    virtual bool Save(const argos_lib::swerve::SwerveModulePositions& homePosition) = 0;

    /**
     * @brief Load home position from persistent storage
     *
     * @return Poisitions from persistent storage or std::nullopt if load failed or no positions were
     *         previously stored
     */
    [[nodiscard]] virtual std::optional<argos_lib::swerve::SwerveModulePositions> Load() = 0;
  };

  /**
   * @brief Interface capable of saving and loading home positions from persistent storage
   *
   * @tparam T Type of the home position.  Should be a units type
   */
  template <class T>
  class HomingStorageInterface {
   public:
    /**
     * @brief Save home position to persistent storage
     *
     * @param homePosition Position to store
     * @return true Save successful
     * @return false Error saving
     */
    virtual bool Save(const T& homePosition) = 0;

    /**
     * @brief Load home position from persistent storage
     *
     * @return Poisition from persistent storage or std::nullopt if load failed or no positions were
     *         previously stored
     */
    [[nodiscard]] virtual std::optional<T> Load() = 0;
  };

}  // namespace argos_lib
