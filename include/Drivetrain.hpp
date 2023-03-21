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

class Drivetrain {
  private:
    /**
     * Vectors of ints that store the motor ports for each side of the
     * drivetrain
     */
    Motor_Group left_motors, right_motors;

    /**
     * Variable to store the diameter of the wheel
     * This value is used to convert values passed in for
     * autonomous functions from inches to degrees to rotate for
     */
    double wheel_diameter;

    /**
     * Variable to hold the width of the drivetrain. This value is used in the
     * turnAngle function
     *
     * As long as the robot's base is rectangular, the turning point of the
     * robot will always be down the robot's front-facing middle. So, this value
     * will be usable no matter the wheel configuration
     */
    double base_width;

    /**
     * The drivePID function is a PID controller for the drivetrain. It sets
     * each side of the drivetrain to move a specified length. It is used as the
     * base for all autonomous functions. Individual functions conduct any
     * calculations needed to get the needed target values before passing those
     * values into drivePID. This means that the PID controller for the
     * drivetrain need only be tuned once
     *
     * @param left_targ: The target length to move to, in inches, for the left
     * side of the drivetrain Can be negative to indicate rotating backwards
     * @param right_targ: The target length to move to, in inches, for the right
     * side of the drivetrain Can be negative to indicate rotating backwards
     */
    void drivePID(double left_targ, double right_targ);

    /**
     * The PID constant values, kP for the proportional constant,
     * kI for the integral constant, and kD for the derivative constant
     */
    double kP, kI, kD;

  public:
    /**
     * The constructor for the TankDrive Class
     *
     * @param leftPorts: A list of the ports for the left side of the drive
     * @param rightPorts: A list of the ports for the right side of the
     * drivetrain
     * @param leftRevs: A list of booleans determining which motor(s) on the
     * left side need to be reversed
     * @param rightRevs: A list of booleans determining which motor(s) on the
     * left side need to be reversed The individual booleans in leftRevs and
     * rightRevs match to the ports for each side of the base that match to the
     * port numbers in the corresponding indeces in leftPorts/rightPorts
     * @param gearset: the motor gearset used in the motors (it is assumed that
     * both motors use the same gearset)
     * @param wD: the diameter of the wheels used
     * @param bW: the distance from the middle of the robot to the wheels
     * @param Pconst: the value of the proportional constant in the PID
     * controller
     * @param Iconst: the value of the integral constant in the PID controller
     * @param Dconst: the value of the derivative constant in the PID controller
     */
    Drivetrain(std::initializer_list<int> leftPorts,
               std::initializer_list<int> rightPorts,
               std::initializer_list<bool> leftRevs,
               std::initializer_list<bool> rightRevs,
               pros::motor_gearset_e_t gearset, double wD, double bW,
               double Pconst, double Iconst, double Dconst);

    /**
     * The driver function allows control of the drivetrain during the opcontrol
     * period, with each side of the drivetrain bound to one of the controller
     * joysticks
     *
     * @param controller the ID of the controller to get joystick values from
     */
    void tank_driver(pros::controller_id_e_t controller);

    void arcade_driver(pros::controller_id_e_t controller);

    /**
     * The setVelocity function manually sets the velocity of each motor group.
     * This really only exists so that in the case of extreme emergency (i.e.
     * all autonomous code has broken for some reason and the team has a match
     * soon), setVelocity can be used to manually control the motors in a
     * timer-based autonomous.
     *
     * @param leftVelo: the velocity to set the leftBase motor group to
     * @param rightVelo: the velocity to set the rightBase motor group to
     */
    void set_velocity(int leftVelo, int rightVelo);

    void set_voltage(int leftVolt, int rightVolt);

    /**
     * The turnAngle function is an autonomous function used to turn the robot
     * to a specific angle, relative to its current position.
     *
     * @param angle: the angle to which to turn. Clockwise is positive,
     * counterclockwise is negative. For example, passing in 45 would make the
     * robot turn 45 degrees to the right (clockwise, from an overhead view)
     */
    void turn_angle(double angle);

    /**
     * The moveStraight function is an autonomous function used to
     * tell the robot to drive forward or backward a given amount
     *
     * @param distance: the distance to travel, in inches. Negative values =
     * backwards
     */
    void move_straight(double distance);
};

#endif /* Drivetrain.hpp */
