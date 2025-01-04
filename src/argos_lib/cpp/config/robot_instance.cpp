/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <fstream>
#include <iostream>

#include "argos_lib/config/config_types.h"
#include "wpi/fs.h"

argos_lib::RobotInstance argos_lib::GetRobotInstance() {
  static const fs::path instanceFilePath{"/home/lvuser/robotInstance"};
  try {
    std::ifstream instanceFile(instanceFilePath, std::ios::in);
    std::string instanceString;
    instanceFile >> instanceString;
    instanceFile.close();
    if (instanceString.rfind("Competition", 0) != std::string::npos) {
      return argos_lib::RobotInstance::Competition;
    }
    if (instanceString.rfind("Practice", 0) != std::string::npos) {
      return argos_lib::RobotInstance::Practice;
    }
  } catch (...) {
    // Error accessing file
  }
  // If load fails, return competition
  std::cout << "[ERROR] Could not read from instance file. Defaulting to competition instance.\n";
  return argos_lib::RobotInstance::Competition;
}
