#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');
inline ez::Piston scrapper('A', false);  // Matchloader/scrapper on brain 3-wire A
inline ez::Piston wing('B', false);          // Wing piston on brain 3-wire B
inline ez::Piston pulldown('C', false);      // Pulldown piston on brain 3-wire C
