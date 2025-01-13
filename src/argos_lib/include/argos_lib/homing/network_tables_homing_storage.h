/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <optional>
#include <string>

#include "argos_lib/general/swerve_utils.h"
#include "argos_lib/homing/homing_interface.h"

namespace argos_lib {
  namespace swerve {
    /**
     * @brief Saves and loads swerve module homes to networkTables
     */
    class NetworkTablesHomingStorage : public SwerveHomeStorageInterface {
     public:
      /**
       * @brief Construct a new Network Tables Homing Storage object
       *
       * @param tableName Network table name
       * @param frontLeftPath Key name for front left module
       * @param frontRightPath Key name for front right module
       * @param rearRightPath Key name for rear right module
       * @param rearLeftPath Key name for rear left module
       */
      NetworkTablesHomingStorage(const std::string& tableName,
                                 const std::string& frontLeftPath,
                                 const std::string& frontRightPath,
                                 const std::string& rearRightPath,
                                 const std::string& rearLeftPath);

      /**
       * @brief Save home positions to network tables
       *
       * @param homePosition Positions to store
       * @return true if successful
       */
      bool Save(const argos_lib::swerve::SwerveModulePositions& homePosition) override;
      /**
       * @brief Load home positions from network tables
       *
       * @return Loaded positions or nullopt if failed
       */
      [[nodiscard]] std::optional<argos_lib::swerve::SwerveModulePositions> Load() override;

     private:
      const std::string m_tableName;
      const std::string m_frontLeftPath;
      const std::string m_frontRightPath;
      const std::string m_rearRightPath;
      const std::string m_rearLeftPath;
    };
  }  // namespace swerve
}  // namespace argos_lib
