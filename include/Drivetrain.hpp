/**
 * \file Drivetrain.hpp
 * This file contains the class declaration for the Drivetrain class.
 * This class represents the drivetrain subsystem. It allows easy
 * control of the motors on the drivetrain in both drivetrain and
 * autonomous
 */

#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include <initializer_list>

#include "Motor_Group.hpp"
#include "pros/misc.h"

class Drivetrain {
  private:
    Motor_Group left_motors, right_motors;

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
     * Function: tank_driver
     *
     * A driver control function in which each side of the drivetrain
     * is controlled its respective joystick Y axis (i.e. the left
     * joystick Y axis reading controls the left motors)
     *
     * @param controller The Controller ID whose joystick to read the value of
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
