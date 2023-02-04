/**
 * \file Roller.hpp
 * This file contains the class declaration for the Roller class, which
 * represents the subsystem used to rotate the rollers.
 */

#ifndef ROLLER_HPP
#define ROLLER_HPP

#include "Motor_Group.hpp"
#include "pros/misc.h"
#include <initializer_list>

class Roller {
  private:
    // The motors controlling the roller mechanism. There should only be one,
    // but if, for whatever reason, more motors need to be added, adding them
    // will be trivial.
    Motor_Group motors;

    // The gear ratio between the motor and the component that rotates the
    // roller. Used to determine how far the roller needs to spin for
    // autonomous. Defined as (gear on motor) / (gear on roller rotation
    // mechanism). Any inverting in the gear train should be accounted for by
    // reversing the motors
    double gear_ratio;

  public:
    /**
     * The constructor for the Roller class
     * \param ports The port numbers for all motors in the group
     * \param revs booleans specifying whether to reverse each motor
     * \param gear_ratio The gear ratio between the motor and the mechanism
     * moving the roller. Any inverting in the gear train should be accounted
     * for by reversing the motors
     */
    Roller(std::initializer_list<int> ports, std::initializer_list<bool> revs,
           double gear_ratio);

    // Rotates the roller clockwise. Used for opcontrol
    void clockwise();

    // Rotates the roller clockwise by a given degree amount. Used for
    // autonomous
    void clockwise(double degrees);

    // Rotates the roller counterclockwise. Used for opcontrol
    void counterclockwise();

    // Rotates the roller counterclockwise by a given degree amount. Used for
    // autonomous
    void counterclockwise(double degrees);

    // Sets the mechanism's motors to stop running
    void stop();

    // Allows for roller control in driver
    void driver(pros::controller_id_e_t controller,
                pros::controller_digital_e_t cw_btn,
                pros::controller_digital_e_t ccw_btn);
};

#endif /* Roller.hpp*/