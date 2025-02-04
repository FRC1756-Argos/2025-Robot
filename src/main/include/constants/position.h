#pragma once

#include <argos_lib/general/angle_utils.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

struct position
{
  units::inch_t elevator_hight;
  units::degree_t arm_angle;
  units::degree_t wrista_angle;
};
