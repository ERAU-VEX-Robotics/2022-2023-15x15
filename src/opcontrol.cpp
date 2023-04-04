#include "main.h"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    flywheel.resume_task();
    bool endgame_primed = false;

    while (true) {
        drive.tank_driver_poly(pros::E_CONTROLLER_MASTER, 1.3,
                               pros::E_CONTROLLER_DIGITAL_RIGHT);
        flywheel.driver(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_A,
                        pros::E_CONTROLLER_DIGITAL_B);
        indexer.driver(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_L1,
                       pros::E_CONTROLLER_DIGITAL_L2);
        intake.driver(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_R1,
                      pros::E_CONTROLLER_DIGITAL_R2);
        roller.driver(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_UP,
                      pros::E_CONTROLLER_DIGITAL_DOWN);

        if (pros::c::controller_get_digital_new_press(
                pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_X))
            endgame_primed = true;
        if (pros::c::controller_get_digital_new_press(
                pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_Y) &&
            endgame_primed)
            pros::c::adi_digital_write('d', true);

        pros::delay(2);
    }
    flywheel.end_task();
}
