#include "Motor_Group.hpp"
#include "pros/motors.h"
#include <vector>

/* The Constructors for Motor_Group*/

Motor_Group::Motor_Group(std::initializer_list<int> ports)
    : motor_ports{ports} {}

Motor_Group::Motor_Group(std::initializer_list<int> ports,
                         std::initializer_list<bool> reverses)
    : motor_ports{ports} {
    set_reversed(reverses);
}

Motor_Group::Motor_Group(std::initializer_list<int> ports,
                         pros::motor_gearset_e_t gearing)
    : motor_ports{ports} {
    set_gearing(gearing);
}

Motor_Group::Motor_Group(std::initializer_list<int> ports,
                         std::initializer_list<bool> reverses,
                         pros::motor_gearset_e_t gearing)
    : motor_ports{ports} {
    set_gearing(gearing);
    set_reversed(reverses);
}

Motor_Group::Motor_Group(std::initializer_list<int> ports,
                         std::initializer_list<bool> reverses,
                         pros::motor_gearset_e_t gearing,
                         pros::motor_encoder_units_e_t encoder_units)
    : motor_ports{ports} {
    set_gearing(gearing);
    set_reversed(reverses);
    set_encoder_units(encoder_units);
}

/**
 * As noted in Motor_Group.hpp, all of these functions simply call
 * their respective PROS C API functions on each motor.
 */

/* Movement Functions */
void Motor_Group::brake(void) {
    for (int p : motor_ports)
        pros::c::motor_brake(p);
}

void Motor_Group::move(int voltage) {
    for (int p : motor_ports)
        pros::c::motor_move(p, voltage);
}

void Motor_Group::move_absolute(double position, int velocity) {
    for (int p : motor_ports)
        pros::c::motor_move_absolute(p, position, velocity);
}

void Motor_Group::move_relative(double position, int velocity) {
    for (int p : motor_ports)
        pros::c::motor_move_relative(p, position, velocity);
}

void Motor_Group::move_velocity(int velocity) {
    for (int p : motor_ports)
        pros::c::motor_move_velocity(p, velocity);
}

void Motor_Group::move_voltage(int voltage) {
    for (int p : motor_ports)
        pros::c::motor_move_voltage(p, voltage);
}

void Motor_Group::modify_profiled_velocity(int velocity) {
    for (int p : motor_ports)
        pros::c::motor_modify_profiled_velocity(p, velocity);
}

/* Telemetry Functions */

std::vector<bool> Motor_Group::are_stopped(void) {
    std::vector<bool> stopped(motor_ports.size(), false);
    for (int i = 0; i < motor_ports.size(); ++i)
        stopped[i] = pros::c::motor_is_stopped(motor_ports[i]);
    return stopped;
}

std::vector<bool> Motor_Group::are_over_current(void) {
    std::vector<bool> over_current(motor_ports.size(), false);
    for (int i = 0; i < motor_ports.size(); ++i)
        over_current[i] = pros::c::motor_is_over_current(motor_ports[i]);
    return over_current;
}

std::vector<bool> Motor_Group::are_over_temp(void) {
    std::vector<bool> over_temp(motor_ports.size(), false);
    for (int i = 0; i < motor_ports.size(); ++i)
        over_temp[i] = pros::c::motor_is_over_temp(motor_ports[i]);
    return over_temp;
}

std::vector<bool> Motor_Group::are_reversed(void) {
    std::vector<bool> reversed(motor_ports.size(), false);
    for (int i = 0; i < motor_ports.size(); ++i)
        reversed[i] = pros::c::motor_is_reversed(motor_ports[i]);
    return reversed;
}

double Motor_Group::get_avg_position(void) {
    double sum = 0;
    for (int p : motor_ports)
        sum += pros::c::motor_get_position(p);
    return sum / motor_ports.size();
}

double Motor_Group::get_avg_velocity(void) {
    double sum = 0;
    for (int p : motor_ports)
        sum += pros::c::motor_get_actual_velocity(p);
    return sum / motor_ports.size();
}

std::vector<pros::motor_brake_mode_e_t> Motor_Group::get_brake_modes(void) {
    std::vector<pros::motor_brake_mode_e_t> brake_modes(
        motor_ports.size(), pros::E_MOTOR_BRAKE_COAST);
    for (int i = 0; i < motor_ports.size(); ++i)
        brake_modes[i] = pros::c::motor_get_brake_mode(motor_ports[i]);
    return brake_modes;
}

std::vector<int> Motor_Group::get_current_limits(void) {
    std::vector<int> current_limits(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        current_limits[i] = pros::c::motor_get_current_limit(motor_ports[i]);
    return current_limits;
}

std::vector<int> Motor_Group::get_current_draws(void) {
    std::vector<int> current_draws(motor_ports.size(), 69);
    for (int i = 0; i < motor_ports.size(); ++i)
        current_draws[i] = pros::c::motor_get_current_draw(motor_ports[i]);
    return current_draws;
}

std::vector<int> Motor_Group::get_directions(void) {
    std::vector<int> directions(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        directions[i] = pros::c::motor_get_direction(motor_ports[i]);
    return directions;
}

std::vector<int> Motor_Group::get_efficiencies(void) {
    std::vector<int> efficiencies(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        efficiencies[i] = pros::c::motor_get_efficiency(motor_ports[i]);
    return efficiencies;
}

std::vector<pros::motor_encoder_units_e_t>
Motor_Group::get_encoder_units(void) {
    std::vector<pros::motor_encoder_units_e_t> encoder_units(
        motor_ports.size(), pros::E_MOTOR_ENCODER_DEGREES);
    for (int i = 0; i < motor_ports.size(); ++i)
        encoder_units[i] = pros::c::motor_get_encoder_units(motor_ports[i]);
    return encoder_units;
}

std::vector<uint32_t> Motor_Group::get_faults(void) {
    std::vector<uint32_t> faults(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        faults[i] = pros::c::motor_get_faults(motor_ports[i]);
    return faults;
}

std::vector<uint32_t> Motor_Group::get_flags(void) {
    std::vector<uint32_t> flags(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        flags[i] = pros::c::motor_get_flags(motor_ports[i]);
    return flags;
}

std::vector<double> Motor_Group::get_positions(void) {
    std::vector<double> positions(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        positions[i] = pros::c::motor_get_position(motor_ports[i]);
    return positions;
}

std::vector<double> Motor_Group::get_power(void) {
    std::vector<double> powers(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        powers[i] = pros::c::motor_get_power(motor_ports[i]);
    return powers;
}

std::vector<double> Motor_Group::get_target_positions(void) {
    std::vector<double> target_positions(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        target_positions[i] =
            pros::c::motor_get_target_position(motor_ports[i]);
    return target_positions;
}

std::vector<int> Motor_Group::get_target_velocities(void) {
    std::vector<int> target_velocities(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        target_velocities[i] =
            pros::c::motor_get_target_velocity(motor_ports[i]);
    return target_velocities;
}

std::vector<double> Motor_Group::get_velocities(void) {
    std::vector<double> velocities(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        velocities[i] = pros::c::motor_get_actual_velocity(motor_ports[i]);
    return velocities;
}

std::vector<double> Motor_Group::get_temperatures(void) {
    std::vector<double> temperatures(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        temperatures[i] = pros::c::motor_get_temperature(motor_ports[i]);
    return temperatures;
}

std::vector<double> Motor_Group::get_torques(void) {
    std::vector<double> torques(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        torques[i] = pros::c::motor_get_torque(motor_ports[i]);
    return torques;
}

std::vector<int> Motor_Group::get_voltages(void) {
    std::vector<int> voltages(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        voltages[i] = pros::c::motor_get_voltage(motor_ports[i]);
    return voltages;
}

std::vector<int> Motor_Group::get_voltage_limits(void) {
    std::vector<int> voltage_limits(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        voltage_limits[i] = pros::c::motor_get_voltage_limit(motor_ports[i]);
    return voltage_limits;
}

std::vector<bool> Motor_Group::get_zero_position_flags(void) {
    std::vector<bool> zero_position_flags(motor_ports.size(), 0);
    for (int i = 0; i < motor_ports.size(); ++i)
        zero_position_flags[i] =
            pros::c::motor_get_zero_position_flag(motor_ports[i]);
    return zero_position_flags;
}

/* Configuration Functions */
void Motor_Group::set_brake_mode(pros::motor_brake_mode_e_t brakeMode) {
    for (int p : motor_ports)
        pros::c::motor_set_brake_mode(p, brakeMode);
}

void Motor_Group::set_current_limits(int limit) {
    for (int p : motor_ports)
        pros::c::motor_set_current_limit(p, limit);
}

void Motor_Group::set_current_limits(const std::vector<int> limits) {
    for (int i = 0; i < motor_ports.size(); ++i)
        pros::c::motor_set_current_limit(motor_ports[i], limits[i]);
}

void Motor_Group::set_encoder_units(
    pros::motor_encoder_units_e_t encoderUnits) {
    for (int p : motor_ports)
        pros::c::motor_set_encoder_units(p, encoderUnits);
}

void Motor_Group::set_gearing(pros::motor_gearset_e_t gearing) {
    for (int p : motor_ports)
        pros::c::motor_set_gearing(p, gearing);
}

void Motor_Group::set_reversed(std::vector<bool> reverses) {
    for (int i = 0; i < motor_ports.size(); ++i)
        pros::c::motor_set_reversed(motor_ports[i], reverses[i]);
}

void Motor_Group::set_voltage_limits(int limit) {
    for (int p : motor_ports)
        pros::c::motor_set_voltage_limit(p, limit);
}

void Motor_Group::set_voltage_limits(const std::vector<int> limits) {
    for (int i = 0; i < motor_ports.size(); ++i)
        pros::c::motor_set_voltage_limit(motor_ports[i], limits[i]);
}

void Motor_Group::set_zero_position(double zero_pos) {
    for (int p : motor_ports)
        pros::c::motor_set_zero_position(p, zero_pos);
}

void Motor_Group::reset_positions(void) {
    for (int p : motor_ports)
        pros::c::motor_tare_position(p);
}

/* Miscellaneous Functions */
void Motor_Group::print_telemetry(uint8_t vals_to_print) {
    if (vals_to_print & E_MOTOR_GROUP_TELEM_PRINT_VELOCITY) {
        printf("Velocities: ");
        std::vector<double> velos = get_velocities();
        for (double d : velos)
            printf("%.2lf ", d);
        printf("\n");
    }
    if (vals_to_print & E_MOTOR_GROUP_TELEM_PRINT_POSITION) {
        printf("Positions: ");
        std::vector<double> poses = get_positions();
        for (double d : poses)
            printf("%.2lf ", d);
        printf("\n");
    }
    if (vals_to_print & E_MOTOR_GROUP_TELEM_PRINT_VOLTAGE) {
        printf("Voltages: ");
        std::vector<int> voltages = get_voltages();
        for (int i : voltages)
            printf("%d ", i);
        printf("\n");
    }
    if (vals_to_print & E_MOTOR_GROUP_TELEM_PRINT_CURRENT) {
        printf("Current Draws: ");
        std::vector<int> currents = get_current_draws();
        for (int i : currents)
            printf("%d ", i);
        printf("\n");
    }
    if (vals_to_print & E_MOTOR_GROUP_TELEM_PRINT_TEMPERATURE) {
        printf("Temperatures: ");
        std::vector<double> temps = get_temperatures();
        for (double d : temps)
            printf("%.2lf ", d);
        printf("\n");
    }
    if (vals_to_print & E_MOTOR_GROUP_TELEM_PRINT_TORQUE) {
        printf("Torques: ");
        std::vector<double> torques = get_torques();
        for (double d : torques)
            printf("%.2lf ", d);
        printf("\n");
    }
}
