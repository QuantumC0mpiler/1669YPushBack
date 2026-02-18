#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern pros::Motor flapstage;
extern ez::PID flapstage_pid;

extern int flapstage_manual_power;
extern bool flapstage_driver_control;

void flapstage_wait();
void flapstage_setup();
