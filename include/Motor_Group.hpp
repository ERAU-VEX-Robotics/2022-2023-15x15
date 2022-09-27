/**
 * \file Motor_Group.hpp
 *
 * The Motor_Group class serves as a useful base for subsystem object design. It
 * allows multiple motors in a subsystem to be treated as "one logical motor".
 * In other words, groups of motors can be sent the same command. It
 * encapsulates the PROS C Motor API by storing a vector of ports for each motor
 * and wrapping almost all PROS C API functions so that they are called on all
 * motors in the group.
 *
 * While the PROS API now offers a Motor Group class, this file was already
 * mostly written at the time of that release, and some of the features (such as
 * individually reversing motors in the group) were not available in the PROS
 * Motor Group
 *
 * All of these functions wrap the PROS C Motor API functions with
 * the same, or nearly the same, name.
 */

#ifndef MOTOR_GROUP_HPP
#define MOTOR_GROUP_HPP

#include <initializer_list>
#include <vector>

#include "api.h"

class Motor_Group {
private:
  /**
   * The ports for all the motors
   * Under no circumstances should the motor ports ever change while
   * the program is running. While motorPorts is already private, it is
   * also declared as a const for the sake of emphasis
   */
  std::vector<int> motorPorts;

public:
  /**
   * @param ports A list of integers representing the ports for each motor.
   * Ports are input like so: {<port1>, <port2>, etc.}
   */
  Motor_Group(std::initializer_list<int> ports);

  /**
   * @param ports A list of integers representing the ports for each motor.
   * Ports are input like so: {<port1>, <port2>, etc.}
   * @param revs A list of whether each motor in the group is reversed
   */
  Motor_Group(std::initializer_list<int> ports,
              std::initializer_list<bool> revs);

  /**
   * @param ports A list of integers representing the ports for each motor.
   * Ports are input like so: {<port1>, <port2>, etc.}
   * @param gearing The new gearing for the motors - of type
   * pros::motor_gearset_e_t. Represents the internal gear cartidge in the
   * motors. Every motor in the group should have the same internal gear
   * cartidge.
   */
  Motor_Group(std::initializer_list<int> ports,
              pros::motor_gearset_e_t gearing);

  /**
   * @param ports A list of integers representing the ports for each motor.
   * Ports are input like so: {<port1>, <port2>, etc.}
   * @param revs A list of whether each motor in the group is reversed
   * @param gearing The new gearing for the motors - of type
   * pros::motor_gearset_e_t. Represents the internal gear cartidge in the
   * motors. Every motor in the group should have the same internal gear
   * cartidge.
   */
  Motor_Group(std::initializer_list<int> ports,
              std::initializer_list<bool> revs,
              pros::motor_gearset_e_t gearing);

  /**
   * @param ports A list of integers representing the ports for each motor.
   * Ports are input like so: {<port1>, <port2>, etc.}
   * @param revs A list of whether each motor in the group is reversed
   * @param gearing The new gearing for the motors - of type
   * pros::motor_gearset_e_t. Represents the internal gear cartidge in the
   * motors. Every motor in the group should have the same internal gear
   * cartidge.
   * @param encoderUnits The new encoder units for the motors - of type
   * pros::motor_encoder_units_e_t
   */
  Motor_Group(std::initializer_list<int> ports,
              std::initializer_list<bool> revs, pros::motor_gearset_e_t gearing,
              pros::motor_encoder_units_e_t encoder_units);

  /*-------------------
   * Movement functions
   *-------------------*/

  /**
   * Function: brake
   * This function wraps the PROS Motor::brake function, which sets a motor's
   * velocity to 0, causing each motor to stop using the current brake mode.
   * This function calls the PROS C function motor_brake() on each motor.
   */
  void brake(void);

  /**
   * Function: move
   * Sets the voltage of the motor from -127 to 127.
   *
   * @param voltage The motor voltage from -127 to 127 - the same as
   * pros::Motor::move
   */
  void move(int voltage);

  /**
   * Function: move_absolute
   * Sets the target absolute position for each motor to move to
   *
   * @param position The absolute position to move to in the motor’s encoder
   * units - same as pros::Motor::move_absolute
   * @param velocity The maximum allowable velocity for the movement - same as
   * pros::Motor::move_absolute
   */
  void move_absolute(double position, int velocity);

  /**
   * Function: move_relative
   * Sets the relative target position for each motor to move to
   *
   * @param position The absolute position to move to in the motor’s encoder
   * units - same as pros::Motor::move_relative
   * @param velocity The maximum allowable velocity for the movement - same as
   * pros::Motor::move_relative
   */
  void move_relative(double position, int velocity);

  /**
   * Function: move_velocity
   * Sets the velocity for each motor
   *
   * @param velocity The new motor velocity from +-100, +-200, or +-600
   * depending on the motor’s gearset - same as PROS Motor::move_velocity
   */
  void move_velocity(int velocity);

  /**
   * Function: move_voltage
   * Sets the voltage for the motor from -12000mV to 12000mV
   *
   * @param voltage The new voltage for the motor from -12000 mV to 12000 mV -
   * same as PROS Motor::move_voltage
   */
  void move_voltage(int voltage);

  /**
   * Function: modify_profiled_velocity
   * This function calls the PROS C API function
   * motor_modify_profiled_velocity on each motor in the group. To quote the
   * PROS docs, modify_profiled_velocity "Changes the output velocity for a
   * profiled movement (move_absolute or move_relative). This will have no
   * effect if the motor is not following a profiled movement."
   *
   * @param velocity The new motor velocity from +-100, +-200, or +-600
   * depending on the motor’s gearset - same as PROS Motor::move_velocity
   */
  void modify_profiled_velocity(int velocity);

  /**-------------------
   * Telemetry Functions
   *--------------------*/

  /**
   * Function: are_stopped
   * This function gets the zero velocity flag for each motor, or whether each
   * motor is stopped (hence the name of the function). 1 means that the given
   * motor is stopped, 0 means that the given motor is moving.
   *
   * @returns A vector containing each motor's zero velocity flag
   */
  std::vector<bool> are_stopped();

  /**
   * Function: are_over_current
   * This function returns the flag for each motor indicating if the motor is
   * drawing more than its maximum current
   *
   * @returns A vector containing each motor's over current limit flag
   */
  std::vector<bool> are_over_current();

  /**
   * Function: are_over_temp
   * This function returns the flag for each motor indicating if the motor's
   * temperature is over its limit
   *
   * @returns A vector containing each motor's over temperature limit flag
   */
  std::vector<bool> are_over_temp();

  /**
   * Function: are_reversed
   * This function returns the operation direction for each motor (i.e. if
   * each motor has had its direction reversed from its default operation
   * directions - indicated by the circular arrow on the v5 motor)
   *
   * @returns A vector containing each motor's reverse status
   */
  std::vector<bool> are_reversed();

  /**
   * Function: get_avg_position
   * This function returns the average current position of every motor
   * in the group, using the internal motor encoders. It uses the
   * PROS motor_get_position function and calculates the average of
   * those values
   *
   * @returns The average encoder position for all motor encoders
   */
  double get_avg_position();

  /**
   * Function: get_brake_modes
   * This function gets the current brake modes for each motor
   *
   * @returns A vector containing each motor's brake mode
   */
  std::vector<pros::motor_brake_mode_e_t> get_brake_modes();

  /**
   * Function: get_current_limits
   * This function gets each motor's current limit in mA. The default is
   * 2500mA
   *
   * @returns A vector containing each motor's active current limit in mA
   */
  std::vector<int> get_current_limits();

  /**
   * Function: get_current_draws
   * This function returns the current drawn by each of the motors in
   * milliAmps. These values may or may not be the same, depending on the load
   * on each motor.
   *
   * @returns A vector containing the current current draws for each motor
   */
  std::vector<int> get_current_draws();

  /**
   * Function: get_directions
   * This function returns the direction of movement for each motor in the
   * group, with +1 representing the positive direction and -1 representing
   * the negative direction. In theory, the directions for each motor should
   * all be the same
   *
   * @returns A vector containing the current directions for each motor
   */
  std::vector<int> get_directions();

  /**
   * Function: get_efficiencies
   * This function gets the efficiencies of the motor in percent.
   * From the PROS motor_get_efficiency documentation: "An efficiency of 100%
   * means that the motor is moving electrically while drawing no electrical
   * power, and an efficiency of 0% means that the motor is drawing power but
   * not moving."
   *
   * @returns A vector containing the motors' efficiencies in percent
   */
  std::vector<int> get_efficiencies();

  /**
   * Function: get_encoder_units
   * This function gets each motor's current encoder units. In theory, this
   * value should be the same for all of the motors.
   *
   * @returns A vector containing each motor's current encoder units
   */
  std::vector<pros::motor_encoder_units_e_t> get_encoder_units();

  /**
   * Function: get_faults
   * This function returns a bitfield representing the faults experienced by
   * each motor. These values should be compared to bitmasks contained in
   * pros::motor_fault_e_t
   *
   * @returns A vector containing each motor's fault bitfields
   */
  std::vector<uint32_t> get_faults();

  /**
   * Function: get_faults
   * This function returns a bitfield representing the flags set by
   * each motor during its operations. These values should be compared to
   * bitmasks contained in pros::motor_flag_e_t
   *
   * @returns A vector containing each motor's fault bitfields
   */
  std::vector<uint32_t> get_flags();

  /**
   * Function: get_gearing
   * This function gets each motor's current internal gearset. In theory, this
   * value should be the same for all of the motors.
   *
   * @returns A vector containing each motor's current internal gearset
   */
  std::vector<pros::motor_gearset_e_t> get_gearing();

  /**
   * Function: get_positions
   * This function returns a vector of all the individual positions
   * for each motor in the group. It uses the motor_get_position
   * function in the PROS C API and packages those values in
   * a vector and returns that.
   *
   * @returns A vector containing the positions of each motor encoder
   */
  std::vector<double> get_positions();
  /**
   * Function: get_power
   * This function returns the power drawn by each motor in Watts
   *
   * @returns A vector containing the power drawn by each motor in Watts
   */
  std::vector<double> get_power();

  /**
   * Function: get_target_positions
   * This functions returns the target positions for the motors. In theory,
   * these values should all be the same.
   *
   * @returns A vector containing the target positions for each motor
   */
  std::vector<double> get_target_positions();

  /**
   * Function: get_target_velocities
   * This functions returns the target velocities for the motors. In theory,
   * these values should all be the same.
   *
   * @returns A vector containing the target velocities for each motor
   */
  std::vector<int> get_target_velocities();

  /**
   * Function: get_velocities
   * This function returns the current velocities of the motors in RPM.
   * These values may or may not be the same, depending on the load on
   * each motor
   *
   * @returns A vector containing the current velocities for each motor
   */
  std::vector<double> get_velocities();

  /**
   * Function: get_temperatures
   * This function returns the temperature of each motor in degress Celsius.
   * The PROS documentation notes that the resolutions for these readings are
   * 5 degrees Celsius. The PROS docs also note that the motor will start to
   * reduce its power when the temperature reading is at least 55 degrees
   * Celsius.
   *
   * @returns A vector containing the temperature of each motor in Celsius
   */
  std::vector<double> get_temperatures();

  /**
   * Function: get_torques
   * This function returns the torque generated by each motor in Newton Meters
   *
   * @returns A vector containing the torque generated by each motor in Newton
   * Meters
   */
  std::vector<double> get_torques();

  /**
   * Function: get_voltages
   * This function returns the voltage delivered to each motor in milliVolts
   *
   * @returns A vector containing the voltage delivered to each motor in
   * milliVolts
   */
  std::vector<int> get_voltages();

  /**
   * Function: get_voltage_limits
   * This function gets each motor's voltage limit, in mV?
   *
   * @returns A vector containing each motor's voltage limit, in mV?
   */
  std::vector<int> get_voltage_limits();

  /**
   * Function: get_zero_position_flags
   * This function returns the zero position flag for each motor, which
   * indicates if a motor is at its zero absolute position. If so, the flag is
   * set to 1. Otherwise, the flag is set to 0.
   *
   * @returns A vector containing each motor's zero position flag
   */
  std::vector<bool> get_zero_position_flags();

  /**------------------------
   * Configuration Functions
   *-------------------------*/

  /**
   * Function: set_brake_mode
   * This function sets the brake mode to be used by all of the motors.
   * It calls the motor_set_brake_mode function on each motor
   *
   * @param brake_mode The new brake mode for the motors - of type
   * pros::motor_brake_mode_e_t
   */
  void set_brake_mode(pros::motor_brake_mode_e_t brake_mode);

  /**
   * Function: set_current_limits
   * This function sets the current limits for all motors to the same value
   *
   * @param limit the new current limit for all motors in milliAmps.
   * The default value is 2500mA
   */
  void set_current_limits(int limit);
  /**
   * Function: set_current_limits
   * This function sets the current limits for each motors to a given value
   *
   * @param limits an vector of limits, one for each motor.
   * The default value is 2500mA
   */
  void set_current_limits(const std::vector<int> limits);

  /**
   * Function: set_encoder_units
   * This function sets the encoder units to be used by all of the motors.
   * It calls the motor_set_encoder_units function on each motor
   *
   * @param encoderUnits The new encoder units for the motors - of type
   * pros::motor_encoder_units_e_t
   */
  void set_encoder_units(pros::motor_encoder_units_e_t encoder_units);

  /**
   * Function: set_gearing
   * This function sets the internal gearing for all of the motors.
   * It calls the motor_set_gearing function on each motor. Every motor in the
   * group should have the same internal gearing.
   *
   * @param gearing The new gearing for the motors - of type
   * pros::motor_gearset_e_t. Represents the internal gear cartidge in the
   * motors
   */
  void set_gearing(pros::motor_gearset_e_t gearing);

  /**
   * Function: set_reversed
   * This function sets the reversed status for each motor individually
   *
   * @param reverses A vector containing the reversed status of each motor as
   * a boolean
   */
  void set_reversed(std::vector<bool> reverses);

  /**
   * Function: set_voltage_limits
   * This function sets the voltage limits for all motors to the same value
   *
   * @param limit the new current limit for all motors in milliAmps.
   */
  void set_voltage_limits(int limit);
  /**
   * Function: set_voltage_limits
   * This function sets the voltage limits for each motor to a given value
   *
   * @param limits an vector of limits, one for each motor.
   */
  void set_voltage_limits(const std::vector<int> limits);

  /**
   * Function: set_zero_position
   * This function sets the zero positions of all motors to the given value.
   * The units for this value are the motor's current encoder units
   *
   * @param zero_pos The new zero position, in the motor's current encoder
   * units
   */
  void set_zero_position(double zero_pos);

  /**
   * Function: reset_positions
   * This function resets the positions of the internal motor encoders.
   * It calls the PROS motor_tare_position function on each motor.
   */
  void reset_positions();
};

#endif /* Motor_Group.hpp */