/**
 * Function definitions for the Intake class
 */

#include "Intake.hpp"
#include "pros/rtos.hpp"
#include <initializer_list>

Intake::Intake(std::initializer_list<int> ports,
               std::initializer_list<bool> reverses)
    : motors(ports, reverses) {
    motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}

void Intake::driver(pros::controller_id_e_t controller,
                    pros::controller_digital_e_t in_button,
                    pros::controller_digital_e_t out_button) {
    if (pros::c::controller_get_digital(controller, in_button))
        in();
    else if (pros::c::controller_get_digital(controller, out_button))
        out();
    else
        motors.move_velocity(0);
}

void Intake::in() { motors.move(127); }

void Intake::out() { motors.move(-127); }

void Intake::turn_degree(double degrees) {
    motors.reset_positions();
    motors.move_relative(degrees, 200);
    while (!(fabs(motors.get_avg_position()) >= fabs(degrees)))
        pros::delay(2);
}

void Intake::print_telemetry(uint8_t vals_to_print) {
    printf("Intake Telemetry\n");
    motors.print_telemetry(vals_to_print);
}