/**
 * \file Drivetrain.hpp
 * This file contains the class declaration for the Drivetrain class.
 * This class represents the drivetrain subsystem. It allows easy
 * control of the motors on the drivetrain in both drivetrain and
 * autonomous
 */

#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include <atomic>
#include <initializer_list>

#include "Motor_Group.hpp"
#include "pros/adi.h"
#include "pros/misc.h"

class Drivetrain {
  private:
    // The motor groups containing each group of motors that power each side of
    // the base
    Motor_Group left_motors, right_motors;

    // The Drivetrain's PID constants, both for moving straight and for turning
    double kP_straight, kP_turn, kI_straight, kI_turn, kD_straight, kD_turn = 0;

    // The threshold for when to set is_settled to true.
    double settled_threshold = 0.1;

    // Variables tracking important information about the drivetrain
    double track_width, tracking_wheel_radius;

    // Atomic variables storing the PID controllers targets
    std::atomic<double> left_targ, right_targ = 0;

    // The PROS task type that contains the PID task for the drivetrain
    pros::task_t pid_task;

    // The ADI shaft encoders on each side of the drive train. These are used to
    // accurately track the rotation of the robot's wheels, allowing us to move
    // the robot with high precision.
    pros::c::adi_encoder_t left_encdr, right_encdr;

    // Boolean tracking which PID constants to use
    std::atomic<bool> use_turn_consts = false;

    // Boolean tracking whether the drivetrain PID has stopped. Used to
    // determine if the drivetrain has completed it's action during the
    // autonomous period.
    std::atomic<bool> is_settled = false;

    // Boolean tracking whether the drivetrain includes encoders
    bool using_encdrs = false;

    /**
     * The PID task function. This function contains a loop that executes the
     * code for PID controllers for each motor group of the drivetrain
     */
    void pid_task_fn();

    /**
     * A static function used to call pid_task_fn. The PROS task system requires
     * that member functions that are passed as task functions must be static. I
     * can't make pid_task_fn static, because it includes references to the
     * Flywheel object. So, I created this function, based on information from:
     * https://www.vexforum.com/t/pros-task-on-member-functions/105000/9
     */
    static void trampoline(void *param);

  public:
    /**
     * The Constructor for the Drivetrain Class
     *
     * @param left_ports A list of motor ports used for the motors
     * 	      on the left side of the drivetrain
     * @param right_ports A list of motor ports used for the motors
     * 	      on the right side of the drivetrain
     * @param left_revs A list of booleans specifying which motors
     * 	      on the left side of the drivetrain are reversed
     * 	      Each boolean is matched with its respective port in
     * 	      left_ports (e.g. for left_ports = {1, 2} and
     * 	      left_revs = {false, true}, the motor in port 2
     * 	      is reversed, while the motor in port 1 isn't.
     * @param right_revs A list of booleans specifying which motors
     * 	      on the right side of the drivetrain are reversed
     * 	      Each boolean is matched with its respective port in
     * 	      right_ports
     */
    Drivetrain(std::initializer_list<int> left_ports,
               std::initializer_list<int> right_ports,
               std::initializer_list<bool> left_revs,
               std::initializer_list<bool> right_revs);

    /**
     * Function: add_adi_encoders
     * This function initializes a pair of ADI (3 wire) shaft encoders for use
     * for the robot during the autonomous period. It also sets a flag to enable
     * use of the initialized encoders in the PID task*/
    void add_adi_encoders(char left_encdr_top_port, char left_encdr_bot_port,
                          bool left_encdr_rev, char right_encdr_top_port,
                          char right_encdr_bot_port, bool right_encdr_rev);

    /**
     * Function: tank_driver
     *
     * A driver control function in which each side of the drivetrain
     * is controlled its respective joystick Y axis (i.e. the left
     * joystick Y axis reading controls the left motors)
     *
     * @param controller The Controller ID whose joystick to read the value
     * of
     */
    void tank_driver(pros::controller_id_e_t controller);

    /**
     * Function: arcade_driver
     *
     * A driver control function in one joystick controls
     * both forward/backward movement and turning.
     *
     * This implementation uses the left joystick
     *
     * The Y axis controls forward/backward
     * The X axis controls turning
     *
     * @param controller The Controller ID whose joystick to read the value of
     */
    void arcade_driver(pros::controller_id_e_t controller);

    /**
     * Functions to set the PID controller constants. Each controller uses the
     * same constants, based on the assumption that any possible difference in
     * the optimal PID constants for each motor group is negligible
     * @param Pconst the new Proportional constant
     * @param Iconst the new Integral constant
     * @param Dconst the new Derivative constant
     */
    // straight constants are used when the robot travels in a straight line,
    // while turn constants are used while turning
    void set_pid_straight_consts(double Pconst, double Iconst, double Dconst);
    void set_pid_turn_consts(double Pconst, double Iconst, double Dconst);

    /**
     * Function: move_straight
     * This function updates the values of the left and right PID targets to
     * make the robot move forward a given number of inches.
     * @param inches  The number of inches to move forward. Negative values
     *                indicate moving backwards.
     */
    void move_straight(double inches);

    /**
     * Function: turn_angle
     * This function updates the values of the left and right PID targets to
     * make the robot turn by a given degree amount clockwise, from a bird's eye
     * view.
     * @param inches  The number of degrees to turn clockwise. Negative values
     *                indicate counterclockwise movement.
     */
    void turn_angle(double angle);

    // Initializes pid_task, starting the Drivetrain PID task
    void init_pid_task();
    // Removes/deletes the Drivetrain PID task. Should always be called at the
    // end of autonomous().
    void end_pid_task();

    // Sets the threshold for declaring whether the drivetrain has settled, i.e.
    // has reached its target position.
    void set_settled_threshold(double threshold);

    /**
     * Function: print_telemetry
     *
     * This function wraps the function calls for print_telemetry
     * for each motor group. See Motor_Group.hpp for more information
     *
     * @param left_vals The bitmap of values to print for the left motor group
     * @param right_vals The bitmap of values to print for the right motor group
     */
    void print_telemetry(uint8_t left_vals, uint8_t right_vals);
};

#endif /* Drivetrain.hpp */
