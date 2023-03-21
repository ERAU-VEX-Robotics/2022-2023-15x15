#include "Indexer.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

#define INDEXER_VELO 60

Indexer::Indexer(std::initializer_list<int> ports,
                 std::initializer_list<bool> revs)
    : motors(ports, revs) {
    motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    motors.set_gearing(pros::E_MOTOR_GEAR_RED);
}

void Indexer::set_rotation(int degrees_to_rotate) {
    this->degrees_to_rotate = degrees_to_rotate;
}

void Indexer::punch_disk() {
    motors.move_velocity(INDEXER_VELO);
    pros::delay(4000);
    motors.move(0);
    pros::delay(1000);
}

void Indexer::driver(pros::controller_id_e_t controller,
                     pros::controller_digital_e_t fire_btn,
                     pros::controller_digital_e_t pullback_btn) {
    if (pros::c::controller_get_digital(controller, fire_btn))
        motors.move_velocity(INDEXER_VELO);
    else if (pros::c::controller_get_digital(controller, pullback_btn))
        motors.move_velocity(-INDEXER_VELO);
    else
        motors.move(0);
}