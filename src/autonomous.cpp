#include "Flywheel.hpp"
#include "externs.hpp"
#include "main.h"
#include "pros/rtos.hpp"

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

    switch (auton_id) {
    case test:
        break;
    case skills_best:
        // Launch pre-loads into low goal
        indexer.punch_disk();
        pros::delay(250);
        indexer.punch_disk();
        pros::delay(250);
        flywheel.pause_task();

        // Drive to roller
        drive.move_straight(24);
        drive.wait_until_settled();
        drive.turn_angle(90);
        drive.wait_until_settled();
        drive.move_straight(2);
        drive.wait_until_settled();

        // Score roller
        intake.in();
        pros::delay(1000);
        intake.stop();

        // Turn to grab disks
        drive.move_straight(-2);
        drive.wait_until_settled();
        drive.turn_angle(135);
        drive.wait_until_settled();
        intake.in();

        // Grab 3 disks
        drive.move_straight(68);
        drive.wait_until_settled();
        intake.stop();
        flywheel.resume_task();

        // Shoot disks
        drive.turn_angle(93);
        drive.wait_until_settled();
        indexer.punch_disk();
        pros::delay(250);
        indexer.punch_disk();
        pros::delay(250);
        indexer.punch_disk();
        pros::delay(250);

        // Go for the stack of 3 disks
        intake.in();
        drive.turn_angle(-93);
        drive.wait_until_settled();
        drive.move_straight(34);
        drive.wait_until_settled();
        pros::delay(2000);
        intake.stop();

        // Launch the three disks
        drive.turn_angle(100);
        drive.wait_until_settled();
        indexer.punch_disk();
        pros::delay(250);
        indexer.punch_disk();
        pros::delay(250);
        indexer.punch_disk();
        pros::delay(250);

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