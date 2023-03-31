#ifndef FLYWHEEL_HPP
#define FLYWHEEL_HPP

/**
 * \file Flywheel.hpp
 *
 * This file contains the class declaration for the Flywheel class.
 * This class represents the physical flywheel used to launch the
 * disks
 */
#include "Motor_Group.hpp"
#include "pros/rtos.h"
#include <atomic>
#include <initializer_list>

#define FLYWHEEL_FAST_TARG 600
#define FLYWHEEL_SLOW_TARG 400

class Flywheel {
  private:
    // The motor group containing all of the motors on the flywheel
    Motor_Group motors;

    std::atomic<int> velocity;

    /**
     * The task function. This function contains a loop that executes the
     * code to run the flywheel's controller in its own task
     */
    void task_fn();

    /**
     * Flywheel controller constants
     *
     * The flywheel controller uses combination of feedforward (i.e. use the
     * target value to set the output) and P (i.e. use the error from the
     * target to determine output) control to set the voltage to the flywheel.
     *
     * See:
     * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
     * as well as
     * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html
     *
     * kS: voltage needed to get the motor to move
     * kV: multiplies the set velocity, "describes how much voltage is needed to
     * hold (or “cruise”) at a given constant velocity" (accounting for forces
     * against the mechanism's movement that increase as velocity increases)
     * kP: Direct multiplier on the current error
     * kD: Multiplier on the difference between the previous and current error
     */
    double kS, kV, kD, kP;

    // The PROS task type that contains the task for the flywheel
    pros::task_t task;

    /**
     * A static function used to call task_fn. The PROS task system requires
     * that member functions that are passed as task functions must be static. I
     * can't make pid_task_fn static, because it includes references to the
     * Flywheel object. So, I created this function, based on information from:
     * https://www.vexforum.com/t/pros-task-on-member-functions/105000/9
     */
    static void trampoline(void *param);

  public:
    /**
     * The Constructor for the Intake Class
     *
     * \param ports A list of ports used by motors on the flywheel
     * \param reverses A list of booleans specifying which motors are reversed
     *             Each element in the list corresponds to the respective
     *             element in ports, i.e. whether or not the motor in the port
     *             specified in the third index of ports is determined by the
     *             third index of revs.
     */
    Flywheel(std::initializer_list<int> ports,
             std::initializer_list<bool> reverses);

    void set_consts(double kS, double kV, double kA, double kP);

    void set_speed_slow();
    void set_speed_fast();

    // Initializes _task, starting the Flywheel task
    void init_task();
    // Pauses the Flywheel task
    void pause_task();
    // Resumes the Flywheel PID task, assuming it was previously paused
    void resume_task();
    // Removes/deletes the Flywheel PID task
    void end_task();

    // Sets the flywheel's target velocity
    void set_target_velo(int velo);

    /**
     * Function: driver
     *
     * This function allows for starting and stopping of the flywheel in driver
     * control, as well as changing the direction
     */
    void driver(pros::controller_id_e_t controller,
                pros::controller_digital_e_t pwr_button,
                pros::controller_digital_e_t rev_button);

    /**
     * Function: set_velocity
     *
     * This function sets the velocity for the motors running the
     * flywheel.
     */
    void set_velocity(int32_t velocity);

    /**
     * Function: set_voltage
     *
     * This function sets the voltage for the motors running the
     * flywheel. Used mainly to determine controller gains
     */
    void set_voltage(int32_t voltage);

    /**
     * Function: stop
     *
     * This function stops the flywheel.
     */
    void stop();

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

#endif /* Flywheel.hpp */