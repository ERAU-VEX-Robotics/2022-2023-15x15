#include "main.h"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    drive.init_pid_task();
    /*
    flywheel.resume_pid_task();

    // After scoring roller, move to opposite roller through the low goal
    drive.move_straight(100);
    drive.wait_until_settled();

    drive.turn_angle(-90);
    drive.wait_until_settled();

    drive.move_straight(108);
    drive.wait_until_settled();

    // Score the roller
    drive.turn_angle(90);
    drive.wait_until_settled();

    drive.move_straight(4);
    drive.wait_until_settled();

    // Back away from roller
    drive.move_straight(-4);
    drive.wait_until_settled();

    // Pick up nearest disk on our half of the field
    drive.turn_angle(135);
    drive.wait_until_settled();

    drive.move_straight(48);
    drive.wait_until_settled();

    // Aim for goal and fire
    drive.turn_angle(80);
    drive.wait_until_settled();

    drive.move_straight(16);
    drive.wait_until_settled();

    flywheel.pause_pid_task();
    */
    pros::c::adi_port_set_config('e', pros::E_ADI_DIGITAL_OUT);

    drive.move_straight(12);
    drive.wait_until_settled();

    pros::c::adi_digital_write('e', true);

    /**/

    drive.end_pid_task();
}