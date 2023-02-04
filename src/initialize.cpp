#include "main.h"
Drivetrain drive({17, 18}, {15, 16}, {true, true}, {false, false});
Intake intake({9}, {true});
Flywheel flywheel({19}, {true});
Conveyor conveyor({20}, {false});
Roller roller({10}, {false}, 1);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    drive.set_drivetrain_dimensions(14.5, 2, 1);
    drive.set_pid_straight_consts(100, 0, 0);
    drive.set_pid_turn_consts(100, 0, 0);
    drive.add_adi_encoders('c', 'd', false, 'g', 'h', true);

    flywheel.set_pid_consts(45, 5, 1);
    flywheel.init_pid_task();
    flywheel.pause_pid_task();
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
