/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include "units/angular_acceleration.h"
#include "units/angular_jerk.h"
#include "units/angular_velocity.h"

namespace controlLoop {
  namespace comp_bot {
    namespace drive {
      struct rotate {
        constexpr static double kP = 137;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kS = 0.0;
        constexpr static double kV = 0.0;
        constexpr static double kA = 0.0;
        constexpr static double kG = 0.0;
        constexpr static int gravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
      };  // namespace rotate
      struct drive {
        constexpr static double kP = 0.3;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kS = 0.0;
        constexpr static double kV = 0.1241;
        constexpr static double kA = 0.0;
        constexpr static double kG = 0.0;
        constexpr static int gravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
      };  // namespace drive
      struct linear_follower {
        constexpr static double kP = 1.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.15;
      };  // namespace linear_follower
      struct rotational_follower {
        constexpr static double kP = 1.0;
        constexpr static double kI = 0.00;
        constexpr static double kD = 0.05;
        constexpr static auto angularVelocity = units::degrees_per_second_t{360};
        constexpr static auto angularAcceleration = units::degrees_per_second_squared_t{360};
      };  // namespace rotational_follower
    }     // namespace drive
    namespace elevator {
      struct elevator {
        constexpr static double kP = 0.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kS = 0.0;
        constexpr static double kV = 0.0;
        constexpr static double kA = 0.0;
        constexpr static double kG = 0.0;
        constexpr static int gravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
      };
      struct arm {
        constexpr static double kP = 30.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kS = 0.5;
        constexpr static double kV = 11.0;
        constexpr static double kA = 0.0;
        constexpr static double kG = 0.0;
        constexpr static int gravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
        constexpr static auto motionMagic_cruiseVelocity = units::angular_velocity::turns_per_second_t{50.0};
        constexpr static auto motionMagic_acceleration = units::angular_acceleration::turns_per_second_squared_t{5.0};
        constexpr static auto motionMagic_jerk = units::angular_jerk::turns_per_second_cubed_t{0.0};
        constexpr static auto motionMagic_expo_kV = ctre::unit::volts_per_turn_per_second_t{5.0};
        constexpr static auto motionMagic_expo_kA = ctre::unit::volts_per_turn_per_second_squared_t{7.5};
      };
      struct wrist {
        constexpr static double kP = 0.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kS = 0.0;
        constexpr static double kV = 0.0;
        constexpr static double kA = 0.0;
        constexpr static double kG = 0.0;
        constexpr static int gravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
      };
    }  // namespace elevator
    namespace climber {
      struct climber {
        constexpr static double kP = 0.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kS = 0.0;
        constexpr static double kV = 0.0;
        constexpr static double kA = 0.0;
        constexpr static double kG = 0.0;
        constexpr static int gravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
      };
    }  // namespace climber
    namespace intake {
      struct intake {
        constexpr static double kP = 0.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kS = 0.0;
        constexpr static double kV = 0.0;
        constexpr static double kA = 0.0;
        constexpr static double kG = 0.0;
        constexpr static int gravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
      };

    }  // namespace intake
  }    // namespace comp_bot
  namespace practice_bot {
    namespace drive {
      using rotate = controlLoop::comp_bot::drive::rotate;
      using drive = controlLoop::comp_bot::drive::drive;
      using linear_follower = controlLoop::comp_bot::drive::linear_follower;
      using rotational_follower = controlLoop::comp_bot::drive::rotational_follower;
    }  // namespace drive
    namespace elevator {
      using elevator = controlLoop::comp_bot::elevator::elevator;
      using arm = controlLoop::comp_bot::elevator::arm;
      using wrist = controlLoop::comp_bot::elevator::wrist;
    }  // namespace elevator
    namespace climber {
      using climber = controlLoop::comp_bot::climber::climber;
    }  // namespace climber
    namespace intake {
      using intake = controlLoop::comp_bot::intake::intake;
    }  // namespace intake
  }    // namespace practice_bot
}  // namespace controlLoop
