#include "Roller.hpp"

Roller::Roller(std::initializer_list<int> ports,
               std::initializer_list<bool> revs, double gear_ratio)
    : motors(ports, revs), gear_ratio(gear_ratio) {
    motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}

void Roller::clockwise() { motors.move(127); }
void Roller::counterclockwise() { motors.move(-127); }
void Roller::stop() { motors.move(0); }

void Roller::clockwise(double degrees) {
    motors.move_relative(degrees / gear_ratio, 200);
    printf("rotation: %lf\n", degrees / gear_ratio);
}

void Roller::counterclockwise(double degrees) {
    motors.move_relative(-degrees / gear_ratio, 200);
}

void Roller::driver(pros::controller_id_e_t controller,
                    pros::controller_digital_e_t cw_btn,
                    pros::controller_digital_e_t ccw_btn) {
    if (pros::c::controller_get_digital(controller, cw_btn))
        clockwise();
    else if (pros::c::controller_get_digital(controller, ccw_btn))
        counterclockwise();
    else
        stop();
}