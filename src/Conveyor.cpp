#include "Conveyor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

Conveyor::Conveyor(std::initializer_list<int> ports,
                   std::initializer_list<bool> revs)
    : motors(ports, revs) {
    motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void Conveyor::forward() { motors.move(127); }
void Conveyor::reverse() { motors.move(-127); }
void Conveyor::stop() { motors.move(0); }

void Conveyor::rotate(double degrees) { motors.move_relative(degrees, 200); }

void Conveyor::driver(pros::controller_id_e_t controller,
                      pros::controller_digital_e_t fwd_btn,
                      pros::controller_digital_e_t rev_btn) {
    if (pros::c::controller_get_digital(controller, fwd_btn))
        forward();
    else if (pros::c::controller_get_digital(controller, rev_btn))
        reverse();
    else
        stop();
}