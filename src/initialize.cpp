#include "Indexer.hpp"
#include "main.h"
Drivetrain drive({11, 12}, {13, 14}, {true, true}, {false, false});
Intake intake({10}, {true});
Flywheel flywheel({20}, {true});
Indexer indexer({19}, {false});
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    // GUI init
    gui_init();

    drive.set_drivetrain_dimensions(14.5, 1.625, 1);
    drive.set_pid_straight_consts(5, 1, 0);
    drive.set_pid_turn_consts(3, 1, 0);
    drive.add_adi_encoders('c', 'd', false, 'g', 'h', false);

    flywheel.set_pid_consts(20, 5, 0.1);
    flywheel.init_pid_task();
    flywheel.pause_pid_task();

    indexer.set_rotation(120);

    pros::c::adi_port_set_config('e', pros::E_ADI_DIGITAL_OUT);
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
