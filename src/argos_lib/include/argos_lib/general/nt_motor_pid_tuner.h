/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <condition_variable>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ctre/phoenix6/core/CoreTalonFX.hpp>

#include "argos_lib/general/nt_subscriber.h"

namespace argos_lib {

  /**
   * @brief Generates a double value to convert sensor position values to physical units represented as a double
   *
   * @tparam Callable Function that generates a units value from a double
   * @param toPhysicalUnitsFunction Conversion function
   * @return double Physical unit value stripped of unit so it's just a double
   */
  template <class Callable>
  constexpr double GetPositionConversionFactor(Callable toPhysicalUnitsFunction) {
    return toPhysicalUnitsFunction(units::angle::turn_t{1}).template to<double>();
  }

  /**
   * @brief Generates a double value to convert sensor velocity values to physical units represented as a double
   *
   * @tparam Callable Function that generates a units value from a double
   * @param toPhysicalUnitsFunction Conversion function
   * @return double Physical unit value stripped of unit so it's just a double
   */
  template <class Callable>
  constexpr double GetVelocityConversionFactor(Callable toPhysicalUnitsFunction) {
    return toPhysicalUnitsFunction(units::angle::turn_t{1.0} / units::time::second_t{1.0}).template to<double>();
  }

  /**
   * @brief Conversion factors to aid displaying sensor values as meaningful numbers
   */
  struct ClosedLoopSensorConversions {
    double position{1.0};  ///< Multiply by this to convert sensor position units to physical units
    double velocity{1.0};  ///< Multiply by this to convert sensor velocity units to physical units
    double setpoint{
        1.0};  ///< Multiply by this to convert sensor setpoint units to physical units (should be the same as either position or velocity)
  };

  /**
   * @brief Allows user to set PID parameters from network tables and update the motor configurations on updates.  Also monitors
   *        status information from motors to aid tuning.
   *
   * @note This runs a background thread to read statuses of motors without calling repeatedly
   */
  class NTMotorPIDTuner {
    using BaseTalon = ctre::phoenix6::hardware::core::CoreTalonFX;

   public:
    /**
     * @brief Construct a new NTMotorPIDTuner object
     *
     * @param tableName Root table in which to make tuning and status keys
     * @param motors Pointers to motors that should be monitored and configured.  Can be one motor if only one motor uses the control loop parameters
     * @param pidSlot PID slot to save tunes into
     * @param sensorConversions Sensor conversion factors to make status values human readable
     */
    NTMotorPIDTuner(const std::string& tableName,
                    std::initializer_list<BaseTalon*> motors,
                    unsigned pidSlot,
                    ClosedLoopSensorConversions sensorConversions = {});

    /**
     * @brief Destroy the NTMotorPIDTuner object
     */
    ~NTMotorPIDTuner();

   private:
    argos_lib::NTSubscriber
        m_updateSubscriber;  ///< Subscriber to manage all updates from user inputs through network tables
    const std::vector<BaseTalon*> m_pMotors;       ///< Motors being configured and monitored
    const unsigned m_pidSlot;                      ///< PID slot index actively used on motors
    std::shared_ptr<nt::NetworkTable> m_pntTable;  ///< Network table containing status and tuning keys
    ClosedLoopSensorConversions
        m_sensorConversions;  ///< Sensor conversion factors used to translate raw sensor readings

    std::mutex m_threadMutex;                ///< Lock to aid notifying thread of stop
    std::condition_variable m_threadStopCv;  ///< Used to notify thread to stop at shutdown
    std::thread m_statusUpdateThread;        ///< Thread monitoring motors

    ctre::phoenix6::configs::SlotConfigs m_activeConfigs;  ///< Current PID configurations

    /**
     * @brief Update statuses from all motors
     */
    void UpdateClosedLoopMonitoringThread();
  };

}  // namespace argos_lib
