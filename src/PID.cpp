#include "PID.hpp"

pros::Motor flapstage(-15);  // FlapStage - PORT15
ez::PID flapstage_pid{0.45, 0.0, 0.0, 0.0, "FlapStage"};

int flapstage_manual_power = 0;
bool flapstage_driver_control = false;

void flapstage_wait() {
  while (flapstage_pid.exit_condition({flapstage}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

void flapstage_task() {
  pros::delay(2000);  // Allow EZ-Template calibration to finish first
  while (true) {
    if (flapstage_driver_control) {
      flapstage.move(flapstage_manual_power);
    } else {
      flapstage.move(flapstage_pid.compute(flapstage.get_position()));
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task FlapStage_Task(flapstage_task);

void flapstage_setup() {
  flapstage.tare_position();
  flapstage_pid.target_set(0);
  flapstage_pid.exit_condition_set(80, 50, 300, 150, 500, 500);
}
