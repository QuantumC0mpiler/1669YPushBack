#include "main.h"
#include "PID.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-8, -9, -10},  // Left Chassis Ports (negative port will reverse it!)
    {16, 17, 18},   // Right Chassis Ports (negative port will reverse it!)

    8,      // IMU Port
    4.125,  // Wheel Diameter
    343);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Intake/scorer motor
pros::Motor roller(12);  // Roller - PORT12

// Tracking wheel configuration for odometry.
constexpr int HORIZ_TRACKER_PORT = 9;
constexpr int VERT_TRACKER_PORT = 10;
constexpr double TRACKER_DIAMETER = 2.75;
constexpr double HORIZ_OFFSET = 4.0;
constexpr double VERT_OFFSET = 4.0;

// Make ports negative if a tracker reports backwards when the robot moves forward/right.
ez::tracking_wheel horiz_tracker(HORIZ_TRACKER_PORT, TRACKER_DIAMETER, HORIZ_OFFSET);
ez::tracking_wheel vert_tracker(VERT_TRACKER_PORT, TRACKER_DIAMETER, VERT_OFFSET);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Register tracking wheels for odometry.
  chassis.odom_tracker_back_set(&horiz_tracker);
  chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);
  chassis.opcontrol_drive_activebrake_set(0.0);
  chassis.opcontrol_curve_default_set(0.0, 0.0);

  // Set the drive to your own constants from autons.cpp
  default_constants();

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();

  // PID subsystem setup for flapstage
  flapstage_setup();

  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

void disabled() {
  // . . .
}

void competition_initialize() {
  // . . .
}

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  ez::as::auton_selector.selected_auton_call();
}

void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());
  }
  ez::screen_print(tracker_value + tracker_width, line);
}

void ez_screen_task() {
  while (true) {
    if (!pros::competition::is_connected()) {
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        if (ez::as::page_blank_is_on(0)) {
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);

          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    } else {
      if (ez::as::page_blank_amount() > 0) {
        ez::as::page_blank_remove_all();
      }
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

void ez_template_extras() {
  if (!pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_X)) {
      chassis.pid_tuner_toggle();
    }

    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    chassis.pid_tuner_iterate();
  } else {
    if (chassis.pid_tuner_enabled()) {
      chassis.pid_tuner_disable();
    }
  }
}

void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  flapstage_driver_control = true;
  flapstage_manual_power = 0;

  // Ensure known startup states
  scrapper.set(false);
  wing.set(false);
  pulldown.set(false);

  constexpr int FULL_SPEED = 127;
  constexpr int INTAKE_ROLLER = FULL_SPEED;
  constexpr int INTAKE_FLAPSTAGE = -FULL_SPEED;  // Opposite sign so both mechanisms intake in same physical direction

  while (true) {
    ez_template_extras();

    // Standard split arcade
    chassis.opcontrol_arcade_standard(ez::SPLIT);

    // Intake/scoring controls
    // Priority: L2 reverse both > R2 both full > R1 intake-only (flapstage)
    int roller_power = 0;
    int flapstage_power = 0;
    if (master.get_digital(DIGITAL_L2)) {
      roller_power = -INTAKE_ROLLER;
      flapstage_power = -INTAKE_FLAPSTAGE;
    } else if (master.get_digital(DIGITAL_R2)) {
      roller_power = INTAKE_ROLLER;
      flapstage_power = INTAKE_FLAPSTAGE;
    } else if (master.get_digital(DIGITAL_R1)) {
      roller_power = 0;
      flapstage_power = INTAKE_FLAPSTAGE;
    }
    roller.move(roller_power);
    flapstage_manual_power = flapstage_power;

    // Pneumatics toggles
    scrapper.button_toggle(master.get_digital(DIGITAL_L1));
    wing.button_toggle(master.get_digital(DIGITAL_B));
    pulldown.button_toggle(master.get_digital(DIGITAL_A));

    pros::delay(ez::util::DELAY_TIME);
  }
}
