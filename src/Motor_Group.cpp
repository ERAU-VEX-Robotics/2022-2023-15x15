#include "Motor_Group.hpp"

/* The Constructors for Motor_Group*/

Motor_Group::Motor_Group(std::initializer_list<int> ports)
    : motorPorts{ports} {}

Motor_Group::Motor_Group(std::initializer_list<int> ports,
                         std::initializer_list<bool> revs)
    : motorPorts{ports} {
  set_reversed(revs);
}

Motor_Group::Motor_Group(std::initializer_list<int> ports,
                         pros::motor_gearset_e_t gearing)
    : motorPorts{ports} {
  set_gearing(gearing);
}

Motor_Group::Motor_Group(std::initializer_list<int> ports,
                         std::initializer_list<bool> revs,
                         pros::motor_gearset_e_t gearing)
    : motorPorts{ports} {
  set_gearing(gearing);
  set_reversed(revs);
}

Motor_Group::Motor_Group(std::initializer_list<int> ports,
                         std::initializer_list<bool> revs,
                         pros::motor_gearset_e_t gearing,
                         pros::motor_encoder_units_e_t encoder_units)
    : motorPorts{ports} {
  set_gearing(gearing);
  set_reversed(revs);
  set_encoder_units(encoder_units);
}

/**
 * As noted in Motor_Group.hpp, all of these functions simply call
 * their respective PROS C API functions on each motor.
 */

/* Movement Functions */
void Motor_Group::brake() {
  for (int p : motorPorts)
    pros::c::motor_brake(p);
}

void Motor_Group::move(int voltage) {
  for (int p : motorPorts)
    pros::c::motor_move(p, voltage);
}

void Motor_Group::move_absolute(double position, int velocity) {
  for (int p : motorPorts)
    pros::c::motor_move_absolute(p, position, velocity);
}

void Motor_Group::move_relative(double position, int velocity) {
  for (int p : motorPorts)
    pros::c::motor_move_relative(p, position, velocity);
}

void Motor_Group::move_velocity(int velocity) {
  for (int p : motorPorts)
    pros::c::motor_move_velocity(p, velocity);
}

void Motor_Group::move_voltage(int voltage) {
  for (int p : motorPorts)
    pros::c::motor_move_voltage(p, voltage);
}

void Motor_Group::modify_profiled_velocity(int velocity) {
  for (int p : motorPorts)
    pros::c::motor_modify_profiled_velocity(p, velocity);
}

/* Telemetry Functions */

std::vector<bool> Motor_Group::are_stopped() {
  std::vector<bool> stopped(motorPorts.size(), false);
  for (int i; i < motorPorts.size(); ++i)
    stopped[i] = pros::c::motor_is_stopped(motorPorts[i]);
  return stopped;
}

std::vector<bool> Motor_Group::are_over_current() {
  std::vector<bool> over_current(motorPorts.size(), false);
  for (int i; i < motorPorts.size(); ++i)
    over_current[i] = pros::c::motor_is_over_current(motorPorts[i]);
  return over_current;
}

std::vector<bool> Motor_Group::are_over_temp() {
  std::vector<bool> over_temp(motorPorts.size(), false);
  for (int i; i < motorPorts.size(); ++i)
    over_temp[i] = pros::c::motor_is_over_temp(motorPorts[i]);
  return over_temp;
}

std::vector<bool> Motor_Group::are_reversed() {
  std::vector<bool> reversed(motorPorts.size(), false);
  for (int i; i < motorPorts.size(); ++i)
    reversed[i] = pros::c::motor_is_reversed(motorPorts[i]);
  return reversed;
}

double Motor_Group::get_avg_position() {
  double sum = 0;
  for (int p : motorPorts)
    sum += pros::c::motor_get_position(p);
  return sum / motorPorts.size();
}

std::vector<pros::motor_brake_mode_e_t> Motor_Group::get_brake_modes() {
  std::vector<pros::motor_brake_mode_e_t> brake_modes(
      motorPorts.size(), pros::E_MOTOR_BRAKE_COAST);
  for (int i; i < motorPorts.size(); ++i)
    brake_modes[i] = pros::c::motor_get_brake_mode(motorPorts[i]);
  return brake_modes;
}

std::vector<int> Motor_Group::get_current_limits() {
  std::vector<int> current_limits(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    current_limits[i] = pros::c::motor_get_current_limit(motorPorts[i]);
  return current_limits;
}

std::vector<int> Motor_Group::get_current_draws() {
  std::vector<int> current_draws(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    current_draws[i] = pros::c::motor_get_current_draw(motorPorts[i]);
  return current_draws;
}

std::vector<int> Motor_Group::get_directions() {
  std::vector<int> directions(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    directions[i] = pros::c::motor_get_direction(motorPorts[i]);
  return directions;
}

std::vector<int> Motor_Group::get_efficiencies() {
  std::vector<int> efficiencies(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    efficiencies[i] = pros::c::motor_get_efficiency(motorPorts[i]);
  return efficiencies;
}

std::vector<pros::motor_encoder_units_e_t> Motor_Group::get_encoder_units() {
  std::vector<pros::motor_encoder_units_e_t> encoder_units(
      motorPorts.size(), pros::E_MOTOR_ENCODER_DEGREES);
  for (int i; i < motorPorts.size(); ++i)
    encoder_units[i] = pros::c::motor_get_encoder_units(motorPorts[i]);
  return encoder_units;
}

std::vector<uint32_t> Motor_Group::get_faults() {
  std::vector<uint32_t> faults(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    faults[i] = pros::c::motor_get_faults(motorPorts[i]);
  return faults;
}

std::vector<uint32_t> Motor_Group::get_flags() {
  std::vector<uint32_t> flags(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    flags[i] = pros::c::motor_get_flags(motorPorts[i]);
  return flags;
}

std::vector<double> Motor_Group::get_positions() {
  std::vector<double> positions(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    positions[i] = pros::c::motor_get_position(motorPorts[i]);
  return positions;
}

std::vector<double> Motor_Group::get_power() {
  std::vector<double> powers(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    powers[i] = pros::c::motor_get_power(motorPorts[i]);
  return powers;
}

std::vector<double> Motor_Group::get_target_positions() {
  std::vector<double> target_positions(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    target_positions[i] = pros::c::motor_get_target_position(motorPorts[i]);
  return target_positions;
}

std::vector<int> Motor_Group::get_target_velocities() {
  std::vector<int> target_velocities(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    target_velocities[i] = pros::c::motor_get_target_velocity(motorPorts[i]);
  return target_velocities;
}

std::vector<double> Motor_Group::get_velocities() {
  std::vector<double> velocities(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    velocities[i] = pros::c::motor_get_actual_velocity(motorPorts[i]);
  return velocities;
}

std::vector<double> Motor_Group::get_temperatures() {
  std::vector<double> temperatures(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    temperatures[i] = pros::c::motor_get_temperature(motorPorts[i]);
  return temperatures;
}

std::vector<double> Motor_Group::get_torques() {
  std::vector<double> torques(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    torques[i] = pros::c::motor_get_torque(motorPorts[i]);
  return torques;
}

std::vector<int> Motor_Group::get_voltages() {
  std::vector<int> voltages(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    voltages[i] = pros::c::motor_get_voltage(motorPorts[i]);
  return voltages;
}

std::vector<int> Motor_Group::get_voltage_limits() {
  std::vector<int> voltage_limits(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    voltage_limits[i] = pros::c::motor_get_voltage_limit(motorPorts[i]);
  return voltage_limits;
}

std::vector<bool> Motor_Group::get_zero_position_flags() {
  std::vector<bool> zero_position_flags(motorPorts.size(), 0);
  for (int i; i < motorPorts.size(); ++i)
    zero_position_flags[i] =
        pros::c::motor_get_zero_position_flag(motorPorts[i]);
  return zero_position_flags;
}

/* Configuration Functions */
void Motor_Group::set_brake_mode(pros::motor_brake_mode_e_t brakeMode) {
  for (int p : motorPorts)
    pros::c::motor_set_brake_mode(p, brakeMode);
}

void Motor_Group::set_current_limits(int limit) {
  for (int p : motorPorts)
    pros::c::motor_set_current_limit(p, limit);
}

void Motor_Group::set_current_limits(const std::vector<int> limits) {
  for (int i; i < motorPorts.size(); ++i)
    pros::c::motor_set_current_limit(motorPorts[i], limits[i]);
}

void Motor_Group::set_encoder_units(
    pros::motor_encoder_units_e_t encoderUnits) {
  for (int p : motorPorts)
    pros::c::motor_set_encoder_units(p, encoderUnits);
}

void Motor_Group::set_gearing(pros::motor_gearset_e_t gearing) {
  for (int p : motorPorts)
    pros::c::motor_set_gearing(p, gearing);
}

void Motor_Group::set_reversed(std::vector<bool> reverses) {
  for (int i = 0; i < motorPorts.size(); ++i)
    pros::c::motor_set_reversed(motorPorts[i], reverses[i]);
}

void Motor_Group::set_voltage_limits(int limit) {
  for (int p : motorPorts)
    pros::c::motor_set_voltage_limit(p, limit);
}

void Motor_Group::set_voltage_limits(const std::vector<int> limits) {
  for (int i; i < motorPorts.size(); ++i)
    pros::c::motor_set_voltage_limit(motorPorts[i], limits[i]);
}

void Motor_Group::set_zero_position(double zero_pos) {
  for (int p : motorPorts)
    pros::c::motor_set_zero_position(p, zero_pos);
}

void Motor_Group::reset_positions() {
  for (int p : motorPorts)
    pros::c::motor_tare_position(p);
}