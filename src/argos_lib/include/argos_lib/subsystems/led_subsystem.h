/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/led/animiation.h>
#include <argos_lib/led/panel.h>
#include <ctre/phoenix/led/CANdle.h>
#include <frc/AddressableLED.h>
#include <frc2/command/SubsystemBase.h>

#include <vector>

namespace argos_lib {
  /**
   * @brief Color and animation status for an individual LED
   */
  struct LEDState {
    bool animated{false};                ///< True indicates this is using a stock animation
    frc::AddressableLED::LEDData color;  ///< Color of LED.  Only used when animated is false

    /**
     * @brief It's an equality comparison operator...
     *
     * @param other LEDState to compare to
     * @return True when both states are equivalent
     */
    bool operator==(const LEDState& other) const;
    /**
     * @brief It's an inequality comparison operator...
     *
     * @param other LEDState to compare to
     * @return True when states are different
     */
    bool operator!=(const LEDState& other) const;

    /**
     * @brief Copy assignment operator
     *
     * @param other New value to assign to self
     * @return LEDState& Reference to this object after assignment
     */
    LEDState& operator=(const frc::AddressableLED::LEDData& other);
  };

  class LEDSubsystem : public frc2::SubsystemBase {
   private:
    // Forward declaration
    struct LEDUpdateGroup;

   public:
    /**
     * @brief Construct a new LEDSubsystem object
     *
     * @param numAuxLEDs Number of LEDs attached to the CANdle
     */
    explicit LEDSubsystem(unsigned numAuxLEDs);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * @brief Set LEDs in attached strip/panel to use stock animation
     *
     * @param animation Animation to assign where led offset 0 is beginning of attached LEDs
     * @param slot Animation slot to use
     */
    void StockAnimateAuxLEDs(ctre::phoenix::led::Animation& animation, int slot);

    /**
     * @brief Set LEDs built into CANdle to use stock animation
     *
     * @param animation Animation to assign
     * @param slot Animation slot to use
     */
    void StockAnimateIntegratedLEDs(ctre::phoenix::led::Animation& animation, int slot);

    /**
     * @brief Set LEDs in attached strip/panel to use custom animation.
     *        The LEDs will be updated on every call of the subsystem periodic function.
     *
     * @param animation Animation to use where led offset 0 is beginning of attached LEDs
     */
    void CustomAnimateAuxLEDs(argos_lib::led::Animation animation);

    /**
     * @brief Set LEDs built into CANdle to use custom animation.
     *        The LEDs will be updated on every call of the subsystem periodic function.
     *
     * @param animation Animation to use
     */
    void CustomAnimateIntegratedLEDs(argos_lib::led::Animation animation);

   private:
    std::vector<LEDState> m_currentLEDs;                        ///< LED status to send as update
    std::vector<LEDState> m_prevLEDs;                           ///< LED status prior to last update
    std::vector<argos_lib::led::Animation> m_customAnimations;  ///< Active animations
    ctre::phoenix::led::CANdle m_controller;                    ///< LED controller instance with attached LEDs

    constexpr static unsigned numIntegratedLEDs = 8;  ///< Number of LEDs addressed before attached LED strip/panel

    /**
     * @brief Get sendable LED block updates to change from prev to current
     *
     * @param prev Old LED state
     * @param current New LED state
     * @return std::vector<LEDUpdateGroup> Minimal set of updates to change from prev to current
     */
    std::vector<LEDUpdateGroup> GetDeltaUpdate(const std::vector<LEDState>& prev, const std::vector<LEDState>& current);

    /**
     * @brief Represents a contiguous block of LEDs that have the same color
     */
    struct LEDUpdateGroup {
      /**
       * @brief Construct a new LEDUpdateGroup with specified color
       *
       * @param startIndex Address of first LED in group
       * @param numLEDs Number of LEDs in group
       * @param color Color of LEDs in group
       */
      LEDUpdateGroup(unsigned startIndex, unsigned numLEDs, frc::AddressableLED::LEDData color)
          : startIndex{startIndex}, numLEDs{numLEDs}, color{color} {};

      unsigned startIndex;                 ///< Address of first LED
      unsigned numLEDs;                    ///< Number of LEDs in group
      frc::AddressableLED::LEDData color;  ///< Color of LEDs
    };
  };
}  // namespace argos_lib
