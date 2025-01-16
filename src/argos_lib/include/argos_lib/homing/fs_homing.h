/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <fstream>
#include <iostream>

#include "argos_lib/general/swerve_utils.h"
#include "argos_lib/homing/homing_interface.h"
#include "units/base.h"
#include "wpi/fs.h"

namespace argos_lib {

  class SwerveFSHomingStorage : public SwerveHomeStorageInterface {
   public:
    /**
     * @brief Construct a new File System Homing Storage object
     *
     * @param swerveHomesPath File path relative to home directory to save into and load from
     */
    explicit SwerveFSHomingStorage(const fs::path& swerveHomesPath);
    /**
     * @brief Save positions as new homes
     *
     * @param homePosition Positions that represent 0 degree module orientations
     * @return true if save successful, false otherwise
     */
    bool Save(const argos_lib::swerve::SwerveModulePositions& homePosition) override;
    /**
     * @brief Load absolute positions that represent 0 degree module orientations
     *
     * @return Saved module positions if they exist, otherwise std::nullopt to indicate failure
     */
    std::optional<argos_lib::swerve::SwerveModulePositions> Load() override;

   private:
    /**
   * @brief Get the path of the file to load from and save to
   *
   * @return fs::path Absolute path of persistent storage file
   */
    fs::path GetFilePath();
    const fs::path m_swerveHomesPath;  ///< Path of persistent storage file relative to home directory
  };

  /**
 * @brief Saves and loads home positions from filesystem
 *
 * @tparam T Type of the home position.  Should be a units type
 */
  template <class T>
  class FSHomingStorage : public HomingStorageInterface<T> {
   public:
    /**
   * @brief Construct a new FSHomingStorage object
   *
   * @param homeFilePath Path to save homes to relative to internally-managed root directory
   */
    explicit FSHomingStorage(const fs::path& homeFilePath) : m_homesPath{homeFilePath} {};

    bool Save(const T& homePosition) override {
      try {
        bool success = true;
        std::ofstream configFile(GetFilePath(), std::ios::out);
        if (configFile.good()) {
          configFile << homePosition.template to<double>();
          if (!configFile.good()) {
            std::cout << "[ERROR] Could not write to config file\n";
            success = false;
          }
        } else {
          std::cout << "[ERROR] Could not open config file\n";
          success = false;
        }
        configFile.close();
        return success;
      } catch (...) {
        // Error accessing file
        std::cout << "[ERROR] Could not write to config file\n";
        return false;
      }
    }

    std::optional<T> Load() override {
      try {
        bool success = true;
        std::ifstream configFile(GetFilePath(), std::ios::in);

        if (configFile.peek() == std::ifstream::traits_type::eof()) {
          return std::nullopt;
        }

        double homePosition;
        configFile >> homePosition;

        configFile.close();
        if (success) {
          return units::make_unit<T>(homePosition);
        } else {
          return std::nullopt;
        }
      } catch (...) {
        // Error accessing file
        std::cout << "[ERROR] Could not read from config file\n";
        return std::nullopt;
      }
    }

   private:
    fs::path GetFilePath() {
      static const fs::path homeDir{"/home/lvuser"};
      const fs::path configFile{homeDir / m_homesPath};

      std::cout << "############# Path: " << configFile << '\n';

      // Create empty file if it doesn't exist yet
      if (!fs::exists(configFile)) {
        fs::create_directories(configFile.parent_path());
        std::ofstream newFile(configFile);
        newFile.close();
      }

      return configFile;
    }

    const fs::path m_homesPath;
  };
}  // namespace argos_lib
