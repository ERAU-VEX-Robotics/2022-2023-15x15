/**
 * \file Intake.hpp
 * This file contains the class declaration for the Intake class.
 * This class represents the subsystem used to collect disks.
 */

#ifndef INTAKE_HPP
#define INTAKE_HPP

#include <initializer_list>

#include "Motor_Group.hpp"
#include "pros/misc.h"

class Intake {
  private:
    Motor_Group motors;

  public:
    /**
     * The Constructor for the Intake Class
     *
     * \param ports A list of ports used by motors on the intake
     * \param reverses A list of booleans specifying which motors are reversed
     *             Each element in the list corresponds to the respective
     *             element in ports, i.e. whether or not the motor in the port
     *             specified in the third index of ports is determined by the
     *             third index of revs.
     */
    Intake(std::initializer_list<int> ports,
           std::initializer_list<bool> reverses);

    /**
     * Function: driver
     *
     * The function used to control the intake in driver control
     * \param controller The controller ID whose buttons to read
     * \param in_button The digital button ID to press to run the motors to
     *                  collect disks
     * \param out_button The digital button ID to press to run the motors to
     *                   expel disks
     */
    void driver(pros::controller_id_e_t controller,
                pros::controller_digital_e_t in_button,
                pros::controller_digital_e_t out_button);

    // Wraps the necessary code to run the motors to collect disks
    void in();
    // Wraps the necessary code to run the motors to expel disks from the intake
    void out();

    /**
     * Function: print_telemetry
     *
     * This function allows for the printing of various telemetry information
     * from the motors in the Motor_Group. It wraps the call to
     * Motor_Group::print_telemetry. See that function for more information
     *
     * \param vals_to_print A bitfield specifying which values to print. See
     *                      Motor_Group::print_telemetry for more information
     */
    void print_telemetry(uint8_t vals_to_print);
};
#endif /* intake.hpp */