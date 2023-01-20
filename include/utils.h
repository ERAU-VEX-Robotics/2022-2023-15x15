/**
 * \file utils.h
 *
 * This file contains function declarations for some commonly used algorithms
 */

#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Function: pid
 *
 * This function is implements the math required for a PID controller. It is
 * intended to be used solely to compute the output power, after the error for a
 * subsystem has been calculated. This means that this function can be called in
 * any subsystem, allowing for minimal repetiton of code. For example, each side
 * of the Drivetrain may have its own PID loop. So, rather than include the math
 * for each loop in a while loop, the while loop can just calculate the error
 * and call this function, then use the output of the function to set the output
 * voltage.
 *
 * \param kP The proportional constant for the controller
 * \param kI The integral constant for the controller
 * \param kD The derivative constant for the controller
 * \param error The error from the target value
 * \param integral A pointer to a double containing the integral value. This
 * must be a pointer so that the integral value can be maintained between
 * function calls.
 * \param prev_error A pointer to the previous error. This must
 * be a pointer for the same reason as integral
 *
 * \returns The output power to set the motors to.
 */
double pid(double kP, double kI, double kD, double error, double *integral,
           double *prev_error);

#ifdef __cplusplus
}
#endif

#endif /* utils.h */