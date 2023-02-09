#include "main.h"
Drivetrain drive({17, 18}, {15, 16}, {true, true}, {false, false});
Intake intake({8}, {true});
Flywheel flywheel({19, 20}, {true, true});
Conveyor conveyor({12}, {false});
Roller roller({10}, {false}, 1);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    drive.set_drivetrain_dimensions(14.5, 1.625, 1);
    drive.set_pid_straight_consts(5, 1, 0);
    drive.set_pid_turn_consts(3, 1, 0);
    drive.add_adi_encoders('c', 'd', false, 'g', 'h', false);

    flywheel.set_pid_consts(50, 8, 1);
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
