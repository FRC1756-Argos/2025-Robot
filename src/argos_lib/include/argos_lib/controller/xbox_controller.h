/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/GenericHID.h>
#include <frc2/command/button/Trigger.h>

#include <array>
#include <vector>

#include "argos_lib/general/debounce_settings.h"
#include "vibration.h"

namespace argos_lib {

  class XboxController : public frc::GenericHID {
   public:
    /**
     * @brief Replaces legacy joystick hand API for WPILib
     */
    enum class JoystickHand { kLeftHand, kRightHand };

    enum class Button {
      kA = 1,
      kB = 2,
      kX = 3,
      kY = 4,
      kBumperLeft = 5,
      kBumperRight = 6,
      kBack = 7,
      kStart = 8,
      kStickLeft = 9,
      kStickRight = 10,
      kLeftTrigger = 11,   ///< virtual button
      kRightTrigger = 12,  ///< virtual button
      kUp = 13,            ///< virtual button
      kRight = 14,         ///< virtual button
      kDown = 15,          ///< virtual button
      kLeft = 16,          ///< virtual button
      COUNT
    };

    /**
     * @brief State of an individual button
     */
    struct UpdateStatus {
      bool pressed = false;          ///< Transitioned from inactive to active
      bool released = false;         ///< Transitioned from active to inactive
      bool debouncePress = false;    ///< Transitioned from inactive to active after debounce applied
      bool debounceRelease = false;  ///< Transitioned from active to inactive after debounce applied
      bool rawActive = false;        ///< Raw button status
      bool debounceActive = false;   ///< Button status after debounce applied
    };

    enum class Axis { kLeftX = 0, kLeftY = 1, kLeftTrigger = 2, kRightTrigger = 3, kRightX = 4, kRightY = 5, COUNT };

    XboxController() = delete;
    /**
     * @brief Construct a new Xbox Controller object connected at port index
     *
     * @param port Index of new controller
     */
    explicit XboxController(int port);

    /**
     * @brief Configure debounce for a specified button
     *
     * @param targetButton Button to configure
     * @param newSettings New debounce configuration
     */
    void SetButtonDebounce(Button targetButton, DebounceSettings newSettings);

    /**
     * @brief Swap all configurations (debounce, etc) between this and other controller.
     *        Useful in conjunction with argos_lib::SwappableControllersSubsystem
     *
     * @param other Controller to swap settings with
     */
    void SwapSettings(XboxController& other);

    /**
     * @brief Get X joystick percent from specified joystick
     *
     * @param hand Left or right joystick
     * @return double Percent [-1,1]
     */
    [[nodiscard]] double GetX(JoystickHand hand) const;
    /**
     * @brief Get Y joystick percent from specified joystick
     *
     * @param hand Left or right joystick
     * @return double Percent [-1,1]
     */
    [[nodiscard]] double GetY(JoystickHand hand) const;
    /**
     * @brief Get percent from specified controller trigger button
     *
     * @param hand Left or right trigger
     * @return double Percent [0,1]
     */
    [[nodiscard]] double GetTriggerAxis(JoystickHand hand) const;

    /**
     * @brief Get the status of button after applying debounce
     *
     * @param buttonIdx Index of requested button
     * @return debounced status
     */
    [[nodiscard]] bool GetDebouncedButton(Button buttonIdx);
    /**
     * @brief Detect if a button just transitioned from inactive to active after applying debounce
     *
     * @param buttonIdx Index of requested button
     * @return true if inactive to active transition occurred
     */
    [[nodiscard]] bool GetDebouncedButtonPressed(Button buttonIdx);
    /**
     * @brief Detect if a button just transitioned from active to inactive after applying debounce
     *
     * @param buttonIdx Index of requested button
     * @return true if active to inactive transition occurred
     */
    [[nodiscard]] bool GetDebouncedButtonReleased(Button buttonIdx);

    /**
     * @brief Get the status of a combination of buttons after applying debounce.
     *        Raw combo button state is active when all buttons are simultaneously
     *        active based on their independent debounce settings.
     *
     * @param buttonCombo Buttons that make up the combination
     * @return debounced status
     */
    [[nodiscard]] bool GetDebouncedButton(std::vector<Button> buttonCombo);
    /**
     * @brief Detect if a combination of buttons just transitioned from inactive to active
     *        after applying debounce.  Raw combo button state is active when all buttons
     *        are simultaneously active based on their independent debounce settings.
     *
     * @param buttonCombo Buttons that make up the combination
     * @return true if combination transition to active occurred
     */
    [[nodiscard]] bool GetDebouncedButtonPressed(std::vector<Button> buttonCombo);
    /**
     * @brief Detect if a combination of buttons just transitioned from active to inactive
     *        after applying debounce.  Raw combo button state is active when all buttons
     *        are simultaneously active based on their independent debounce settings.
     *
     * @param buttonCombo Buttons that make up the combination
     * @return true if combination transition to inactive occurred
     */
    [[nodiscard]] bool GetDebouncedButtonReleased(std::vector<Button> buttonCombo);

    /**
     * @brief Get the status of button
     *
     * @param buttonIdx Index of requested button
     * @return Status ignoring any debounce
     */
    [[nodiscard]] bool GetRawButton(Button buttonIdx);
    /**
     * @brief Detect if a button just transitioned from inactive to active
     *
     * @param buttonIdx Index of requested button
     * @return true if inactive to active transition occurred
     */
    [[nodiscard]] bool GetRawButtonPressed(Button buttonIdx);
    /**
     * @brief Detect if a button just transitioned from active to inactive
     *
     * @param buttonIdx Index of requested button
     * @return true if active to inactive transition occurred
     */
    [[nodiscard]] bool GetRawButtonReleased(Button buttonIdx);

    /**
     * @brief Get the status of a combination of buttons ignoring debounce.
     *        Raw combo button state is active when all buttons are simultaneously
     *        active.
     *
     * @param buttonCombo Buttons that make up the combination
     * @return Status ignoring any debounce
     */
    [[nodiscard]] bool GetRawButton(std::vector<Button> buttonCombo);
    /**
     * @brief Detect if a combination of buttons just transitioned from inactive to active.
     *        Raw combo button state is active when all buttons are simultaneously active.
     *
     * @param buttonCombo Buttons that make up the combination
     * @return true if combination transition to active occurred
     */
    [[nodiscard]] bool GetRawButtonPressed(std::vector<Button> buttonCombo);
    /**
     * @brief Detect if a combination of buttons just transitioned from active to inactive.
     *        Raw combo button state is active when all buttons are simultaneously active.
     *
     * @param buttonCombo Buttons that make up the combination
     * @return true if combination transition to inactive occurred
     */
    [[nodiscard]] bool GetRawButtonReleased(std::vector<Button> buttonCombo);

    /**
     * @brief Get the active vibration model
     *
     * @return Active vibration model
     */
    VibrationModel GetVibration() const;

    /**
     * @brief Sets a new vibration pattern and updates vibration output based on that new model
     *
     * @param newVibrationModel Model to generate vibration output
     */
    void SetVibration(VibrationModel newVibrationModel);

    /**
     * @brief Update vibration output based on current vibration model
     */
    void UpdateVibration();

    /**
     * @brief Determines the new status of a button.  This is used by the other
     *        status retrieval functions.
     *
     * @param buttonIdx Index of button to update
     * @return UpdateStatus Full button state
     */
    UpdateStatus UpdateButton(Button buttonIdx);

    /**
     * @brief Generates a trigger that is true when button is true
     *
     * @param button Button to monitor
     * @return frc2::Trigger Trigger to monitor button
     */
    [[nodiscard]] frc2::Trigger TriggerRaw(Button button);

    /**
     * @brief Generates a trigger that is true when all buttons in buttonCombo are true
     *
     * @param buttonCombo Buttons to monitor
     * @return frc2::Trigger Trigger to monitor buttons
     */
    [[nodiscard]] frc2::Trigger TriggerRaw(std::vector<Button> buttonCombo);

    /**
     * @brief Trigger when any of the selected buttons' raw value is true
     *
     * @param buttonCombo Buttons to check
     * @return frc2::Trigger Trigger indicating any button raw value is true
     */
    [[nodiscard]] frc2::Trigger TriggerRawAnyOf(std::vector<Button> buttonCombo);

    /**
     * @brief Trigger when all of the selected buttons' raw values are true
     *
     * @param buttonCombo Buttons to check
     * @return frc2::Trigger Trigger indicating all button raw values are true
     */
    [[nodiscard]] frc2::Trigger TriggerRawAllOf(std::vector<Button> buttonCombo);

    /**
     * @brief Trigger when none of the selected buttons' raw value is true
     *
     * @param buttonCombo Buttons to check
     * @return frc2::Trigger Trigger indicating none button raw value is true
     */
    [[nodiscard]] frc2::Trigger TriggerRawNoneOf(std::vector<Button> buttonCombo);

    /**
     * @brief Trigger when exactly one of the selected buttons' raw value is true
     *
     * @param buttonCombo Buttons to check
     * @return frc2::Trigger Trigger indicating one button's raw value is true
     */
    [[nodiscard]] frc2::Trigger TriggerRawOneOf(std::vector<Button> buttonCombo);

    /**
     * @brief Generates a trigger that is true when debounced button is true
     *
     * @param button Button to monitor
     * @return frc2::Trigger Trigger to monitor debounced button
     */
    [[nodiscard]] frc2::Trigger TriggerDebounced(Button button);

    /**
     * @brief Generates a trigger that is true when all debounced buttons in buttonCombo are true
     *
     * @param buttonCombo Buttons to monitor
     * @return frc2::Trigger Trigger to monitor debounced buttons
     */
    [[nodiscard]] frc2::Trigger TriggerDebounced(std::vector<Button> buttonCombo);

    /**
     * @brief Trigger when any of the selected buttons' debounced value is true
     *
     * @param buttonCombo Buttons to check
     * @return frc2::Trigger Trigger indicating any button debounced value is true
     */
    [[nodiscard]] frc2::Trigger TriggerDebouncedAnyOf(std::vector<Button> buttonCombo);

    /**
     * @brief Trigger when all of the selected buttons' debounced values are true
     *
     * @param buttonCombo Buttons to check
     * @return frc2::Trigger Trigger indicating all button debounced values are true
     */
    [[nodiscard]] frc2::Trigger TriggerDebouncedAllOf(std::vector<Button> buttonCombo);

    /**
     * @brief Trigger when none of the selected buttons' debounced value is true
     *
     * @param buttonCombo Buttons to check
     * @return frc2::Trigger Trigger indicating none button debounced value is true
     */
    [[nodiscard]] frc2::Trigger TriggerDebouncedNoneOf(std::vector<Button> buttonCombo);

    /**
     * @brief Trigger when exactly one of the selected buttons' debounced value is true
     *
     * @param buttonCombo Buttons to check
     * @return frc2::Trigger Trigger indicating one button's debounced value is true
     */
    [[nodiscard]] frc2::Trigger TriggerDebouncedOneOf(std::vector<Button> buttonCombo);

   private:
    /**
     * @brief Parsed directional pad button states
     */
    struct DPadButtons {
      bool up = false;     ///< Up active (including adjacent diagonals)
      bool right = false;  ///< Right active (including adjacent diagonals)
      bool down = false;   ///< Down active (including adjacent diagonals)
      bool left = false;   ///< Left active (including adjacent diagonals)
    };

    /**
     * @brief Convert POV angle to usable DPad button values
     *
     * @return DPadButtons Active status for each cardinal direction
     */
    DPadButtons GetPOVButtons();

    /**
     * @brief Trigger when any of the selected buttons is true according to the supplied buttonGetterFunc
     *
     * @param buttonCombo Buttons to check
     * @param buttonGetterFunc Function to evaluate to determine button state
     * @return frc2::Trigger Trigger indicating any button is true
     */
    [[nodiscard]] frc2::Trigger TriggerAnyOf(std::vector<Button> buttonCombo,
                                             std::function<bool(Button)> buttonGetterFunc);

    /**
     * @brief Trigger when all of the selected buttons is true according to the supplied buttonGetterFunc
     *
     * @param buttonCombo Buttons to check
     * @param buttonGetterFunc Function to evaluate to determine button state
     * @return frc2::Trigger Trigger indicating all buttons are true
     */
    [[nodiscard]] frc2::Trigger TriggerAllOf(std::vector<Button> buttonCombo,
                                             std::function<bool(Button)> buttonGetterFunc);

    /**
     * @brief Trigger when none of the selected buttons is true according to the supplied buttonGetterFunc
     *
     * @param buttonCombo Buttons to check
     * @param buttonGetterFunc Function to evaluate to determine button state
     * @return frc2::Trigger Trigger indicating no button is true
     */
    [[nodiscard]] frc2::Trigger TriggerNoneOf(std::vector<Button> buttonCombo,
                                              std::function<bool(Button)> buttonGetterFunc);

    /**
     * @brief Trigger when exactly one of the selected buttons is true according to the supplied buttonGetterFunc
     *
     * @param buttonCombo Buttons to check
     * @param buttonGetterFunc Function to evaluate to determine button state
     * @return frc2::Trigger Trigger indicating one button is true
     */
    [[nodiscard]] frc2::Trigger TriggerOneOf(std::vector<Button> buttonCombo,
                                             std::function<bool(Button)> buttonGetterFunc);

    constexpr static double analogTriggerThresh = 0.5;  ///< Percent trigger pressed to consider as a button press

    std::array<DebounceSettings, static_cast<int>(Button::COUNT)> m_buttonDebounceSettings;
    std::array<bool, static_cast<int>(Button::COUNT)> m_buttonDebounceStatus;
    std::array<bool, static_cast<int>(Button::COUNT)> m_rawButtonStatus;
    std::array<std::chrono::time_point<std::chrono::steady_clock>, static_cast<int>(Button::COUNT)>
        m_buttonDebounceTransitionTime;  ///< Time when new value was first seen

    VibrationModel m_vibrationModel;  ///< Active vibration model
  };

}  // namespace argos_lib
