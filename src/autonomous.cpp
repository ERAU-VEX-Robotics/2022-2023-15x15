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
    drive.init_pid_task();
    drive.move_straight(0);

    drive.move_straight(24);
    drive.wait_until_settled();

    drive.turn_angle(90);
    drive.wait_until_settled();

    drive.move_straight(-2);
    drive.wait_until_settled();

    drive.turn_angle(-90);
    drive.wait_until_settled();

    drive.move_straight(-12);
    drive.wait_until_settled();

    drive.turn_angle(45);
    drive.wait_until_settled();

    drive.turn_angle(45);
    drive.wait_until_settled();

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

        // Move to the roller and score it
        drive.move_straight(-20);
        drive.wait_until_settled();
        drive.turn_angle(90);
        drive.wait_until_settled();

        drive.move_straight(-7);
        drive.wait_until_settled();

        roller.counterclockwise(120);

        // Fire preloads into high goal
        drive.move_straight(4);
        drive.wait_until_settled();
        drive.turn_angle(20);
        drive.wait_until_settled();

        indexer.punch_disk();
        intake.in();
        pros::delay(2000);
        intake.stop();
        indexer.punch_disk();
        pros::delay(200);
        flywheel.pause_task();

        drive.turn_angle(-60);
        drive.wait_until_settled();
        drive.move_straight(60);
        intake.in();
        flywheel.resume_task();
        drive.wait_until_settled();

        drive.turn_angle(90);
        drive.wait_until_settled();
        indexer.punch_disk();
        indexer.punch_disk();

        break;
    case none:
        break;
    default:
        break;
    }

    drive.end_pid_task();
    flywheel.pause_task();
}