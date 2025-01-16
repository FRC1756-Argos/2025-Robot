/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/homing/network_tables_homing_storage.h"

#include <networktables/NetworkTableInstance.h>

using argos_lib::swerve::NetworkTablesHomingStorage;

NetworkTablesHomingStorage::NetworkTablesHomingStorage(const std::string& tableName,
                                                       const std::string& frontLeftPath,
                                                       const std::string& frontRightPath,
                                                       const std::string& rearRightPath,
                                                       const std::string& rearLeftPath)
    : m_tableName{tableName}
    , m_frontLeftPath{frontLeftPath}
    , m_frontRightPath{frontRightPath}
    , m_rearRightPath{rearRightPath}
    , m_rearLeftPath{rearLeftPath} {
  auto ntInstance{nt::NetworkTableInstance::GetDefault()};
  auto ntTable{ntInstance.GetTable(m_tableName)};
  ntTable->SetPersistent(m_frontLeftPath);
  ntTable->SetPersistent(m_frontRightPath);
  ntTable->SetPersistent(m_rearRightPath);
  ntTable->SetPersistent(m_rearLeftPath);
}

bool NetworkTablesHomingStorage::Save(const argos_lib::swerve::SwerveModulePositions& homePosition) {
  auto ntInstance{nt::NetworkTableInstance::GetDefault()};
  auto ntTable{ntInstance.GetTable(m_tableName)};

  ntTable->PutNumber(m_frontLeftPath, homePosition.FrontLeft.to<double>());
  ntTable->PutNumber(m_frontRightPath, homePosition.FrontRight.to<double>());
  ntTable->PutNumber(m_rearRightPath, homePosition.RearRight.to<double>());
  ntTable->PutNumber(m_rearLeftPath, homePosition.RearLeft.to<double>());

  return true;
}

std::optional<argos_lib::swerve::SwerveModulePositions> NetworkTablesHomingStorage::Load() {
  auto ntInstance{nt::NetworkTableInstance::GetDefault()};
  auto ntTable{ntInstance.GetTable(m_tableName)};
  // Read positions are in degrees
  return argos_lib::swerve::SwerveModulePositions{
      units::make_unit<units::degree_t>(ntTable->GetNumber(m_frontLeftPath, 0)),
      units::make_unit<units::degree_t>(ntTable->GetNumber(m_frontRightPath, 0)),
      units::make_unit<units::degree_t>(ntTable->GetNumber(m_rearRightPath, 0)),
      units::make_unit<units::degree_t>(ntTable->GetNumber(m_rearLeftPath, 0))};
}
