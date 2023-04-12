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

    // Drive PID tuning programs
    // Straight
    /*
    drive.move_straight(48);
    drive.wait_until_settled();

    drive.move_straight(-12);
    drive.wait_until_settled();

    drive.move_straight(6);
    drive.wait_until_settled();

    drive.move_straight(-24);
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
    drive.move_straight(-19.5);
    drive.wait_until_settled();
    drive.turn_angle(90);
    drive.wait_until_settled();

    flywheel.set_target_velo(550);

    drive.move_straight(-5.1);
    drive.wait_until_settled();

    roller.counterclockwise(120);

    // Fire preloads into high goal
    drive.move_straight(4);
    drive.wait_until_settled();
    drive.turn_angle(10);
    drive.wait_until_settled();

    indexer.punch_disk();
    pros::delay(1000);
    indexer.punch_disk();
    pros::delay(1000);
    flywheel.pause_task();

    drive.turn_angle(90);
    drive.wait_until_settled();
    drive.move_straight(-1);
    drive.wait_until_settled();
    drive.turn_angle(35);
    drive.wait_until_settled();

    intake.in();
    flywheel.resume_task();
    flywheel.set_target_velo(425);

    drive.move_straight(-16);
    drive.wait_until_settled();
    drive.turn_angle(-10);
    drive.wait_until_settled();
    drive.move_straight(-16);
    drive.wait_until_settled();
    pros::delay(1000);

    drive.move_straight(-16);
    drive.wait_until_settled();
    drive.turn_angle(5);
    drive.wait_until_settled();
    drive.move_straight(-16);
    drive.wait_until_settled();

    drive.turn_angle(-95);
    drive.wait_until_settled();
    pros::delay(3000);
    intake.stop();
    indexer.punch_disk();
    pros::delay(2000);
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

    drive.end_pid_task();
    flywheel.pause_task();
}