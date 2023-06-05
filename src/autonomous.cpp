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
    flywheel.resume_task();
    drive.resume_pid_task();

    drive.set_voltage_limit(6750);

    // Drive PID tuning programs
    // Straight
    /*
    drive.move_straight(72);
    drive.wait_until_settled();

    drive.move_straight(-60);
    drive.wait_until_settled();

    drive.move_straight(48);
    drive.wait_until_settled();

    drive.move_straight(-36);
    drive.wait_until_settled();

    drive.move_straight(24);
    drive.wait_until_settled();

    drive.move_straight(-12);
    drive.wait_until_settled();

    drive.move_straight(6);
    drive.wait_until_settled();

    drive.move_straight(-3);
    drive.wait_until_settled();

    */

    // Turn
    /*
    drive.turn_angle(180);
    drive.wait_until_settled();

    drive.turn_angle(-150);
    drive.wait_until_settled();

    drive.turn_angle(120);
    drive.wait_until_settled();

    drive.turn_angle(-90);
    drive.wait_until_settled();

    drive.turn_angle(60);
    drive.wait_until_settled();

    drive.turn_angle(-30);
    drive.wait_until_settled();
    */

    // Move to the roller and score it
    flywheel.set_target_velo(520);

    drive.move_straight(-24);
    drive.wait_until_settled();
    drive.turn_angle(90);
    drive.wait_until_settled();

    drive.move_straight(-10);
    drive.wait_until_settled();

    roller.counterclockwise(120);

    // Fire preloads into high goal
    drive.move_straight(4);
    drive.wait_until_settled();
    drive.turn_angle(12);
    drive.wait_until_settled();
    pros::delay(1500);

    indexer.punch_disk();
    pros::delay(1000);
    indexer.punch_disk();
    pros::delay(1000);
    flywheel.pause_task();

    // Pick up 3 disks in middle
    drive.turn_angle(125);
    drive.wait_until_settled();

    pros::c::adi_digital_write('b', true);

    pros::delay(5000);

    // Close solenoids for disk guides
    pros::c::adi_digital_write('b', false);

    drive.move_straight(-5);
    drive.wait_until_settled();

    drive.turn_angle(-8);
    drive.wait_until_settled();

    intake.in();
    flywheel.resume_task();
    flywheel.set_target_velo(470);

    drive.move_straight(-60);
    drive.wait_until_settled();

    // Fire 3 disks
    drive.turn_angle(-80);
    drive.wait_until_settled();
    pros::delay(3000);
    intake.stop();
    indexer.punch_disk();
    indexer.punch_disk();
    indexer.punch_disk();

    switch (auton_id) {
    case test:

        break;
    case skills_best:

        break;
    case skills_real:
        break;
    case match_best:
        break;
    case match_real:
        break;
    case none:
        break;
    default:
        break;
    }

    drive.pause_pid_task();
    flywheel.pause_task();
}