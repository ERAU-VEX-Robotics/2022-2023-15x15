#include "main.h"

Drivetrain drive({20, 19, 18, 17, 16}, {10, 9, 8, 7, 6},
                 {true, false, true, false, true},
                 {false, true, false, true, false});
Intake intake({15, 14}, {true, false});
Flywheel flywheel({5}, {false});
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    flywheel.set_pid_consts(50, 0.1, 1);
    flywheel.init_pid_task();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
