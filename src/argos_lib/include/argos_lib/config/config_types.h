/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <string_view>

namespace argos_lib {
  /**
   * @brief Differentiates between practice robot and competition robot
   */
  enum struct RobotInstance {
    Competition,  ///< Competition robot
    Practice      ///< Practice robot
  };

  /**
   * @brief Detect robot instance
   *
   * @return RobotInstance
   */
  RobotInstance GetRobotInstance();

  struct CANAddress {
    int address;
    std::string_view busName;

    CANAddress() = delete;
    constexpr CANAddress(int address, const std::string_view& busName = "rio") : address(address), busName(busName) {}
  };

  /// @brief Gets integer address from CANAddress objects based on robot instance
  /// @param compAddress The competiton robot CANAddress object
  /// @param practiceAddress The practice robot CANAddress object
  /// @param instance Current instance of robot
  /// @return returns compAddress.addr if instance is competition, practiceAddress.addr if instance is practice
  static inline int GetCANAddr(const argos_lib::CANAddress& compAddress,
                               const argos_lib::CANAddress& practiceAddress,
                               argos_lib::RobotInstance instance) {
    return instance == argos_lib::RobotInstance::Competition ? compAddress.address : practiceAddress.address;
  }

  /// @brief Gets std::string_view bus name from CANAddress objects based on robot instance
  /// @param compAddress The competiton robot CANAddress object
  /// @param practiceAddress The practice robot CANAddress object
  /// @param instance Current instance of robot
  /// @return returns compAddress.busName if instance is competition, practiceAddress.busName if instance is practice
  static inline std::string_view GetCANBus(const argos_lib::CANAddress& compAddress,
                                           const argos_lib::CANAddress& practiceAddress,
                                           argos_lib::RobotInstance instance) {
    return instance == argos_lib::RobotInstance::Competition ? compAddress.busName : practiceAddress.busName;
  }

}  // namespace argos_lib
