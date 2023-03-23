#include "Indexer.hpp"
#include "main.h"
Drivetrain drive({11, 12}, {13, 16}, {true, true}, {false, false});
Intake intake({9}, {false});
Flywheel flywheel({18}, {true});
Indexer indexer({19}, {false});
Roller roller({5}, {true}, 1);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    // GUI init
    gui_init();

    drive.set_drivetrain_dimensions(12.5, 1.625, 1);
    drive.set_pid_straight_consts(40, 0.01, 0);
    drive.set_pid_turn_consts(20, 0.01, 0);
    drive.set_settled_threshold(10);

    flywheel.set_speed_slow();
    flywheel.init_task();
    flywheel.pause_task();

    pros::c::adi_port_set_config('d', pros::E_ADI_DIGITAL_OUT);
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
