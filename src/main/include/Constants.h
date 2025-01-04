/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/PneumaticsModuleType.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <string>

#include "constants/addresses.h"
#include "constants/control_loops.h"
#include "constants/encoders.h"
#include "constants/feature_flags.h"
#include "constants/interpolation_maps.h"
#include "constants/measure_up.h"
#include "constants/motors.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

namespace speeds {
  namespace drive {
    constexpr double aimBotMaxBias = 0.6;
    constexpr double aimBotThresh = 0.1;
    constexpr double lateralInertialWeight = 15.0;
    constexpr double medialInertialWeight = 5.0;
    constexpr double aimSpeedReductionPct = 0.4;
    constexpr double passSpeedReductionPct = 0.5;
    constexpr double generalSpeedReductionPct = 0.75;
    constexpr double rotationalProportionality = 0.011;
    constexpr units::velocity::feet_per_second_t maxAngular = 12_fps;
  }  // namespace drive
}  // namespace speeds

namespace indexes {
  namespace swerveModules {
    constexpr char frontLeftIndex = 0;
    constexpr char frontRightIndex = 1;
    constexpr char backRightIndex = 2;
    constexpr char backLeftIndex = 3;
  }  // namespace swerveModules
}  // namespace indexes

namespace paths {
  const std::string swerveHomesPath = "homes/swerveHomes";
}  // namespace paths

namespace networkTables {
  namespace swerveHomes {
    const std::string tableKey = "Argos";
    namespace keys {
      const std::string flHome = "swerveHomes/flHome";
      const std::string frHome = "swerveHomes/frHome";
      const std::string brHome = "swerveHomes/brHome";
      const std::string blHome = "swerveHomes/blHome";

      const std::string flHomeFullPath = "swerveHomes/flHome";
      const std::string frHomeFullPath = "swerveHomes/frHome";
      const std::string brHomeFullPath = "swerveHomes/brHome";
      const std::string blHomeFullPath = "swerveHomes/blHome";
    }  // namespace keys
  }    // namespace swerveHomes
}  // namespace networkTables

namespace leds {
  // Length of request animation in seconds
  constexpr units::time::second_t requestLen = 2_s;
}  // namespace leds
namespace camera {
  constexpr char reflectivePipeline = 1;
  constexpr char aprilTagPipeline = 0;
  constexpr int horizontalPixelResolution = 320;
  constexpr int verticalPixelResolution = 240;
  constexpr auto horizontalAngleResolution = 63.3_deg;
  constexpr auto halfhorizontalAngleResolution = horizontalAngleResolution / 2;
  constexpr auto verticalAngleResolution = 49.7_deg;
}  // namespace camera

/// @brief Designate the threshold approach direction
enum class ApproachDirection { Increasing, Decreasing };
