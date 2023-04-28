#include "Indexer.hpp"

#define INDEXER_VELO 200
#define INDEXER_ROTATION 720

Indexer::Indexer(std::initializer_list<int> ports,
                 std::initializer_list<bool> revs)
    : motors(ports, revs) {
    motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    motors.set_gearing(pros::E_MOTOR_GEAR_GREEN);
}

void Indexer::set_rotation(int degrees_to_rotate) {
    this->degrees_to_rotate = degrees_to_rotate;
}

void Indexer::punch_disk() {
    motors.reset_positions();
    motors.move_relative(INDEXER_ROTATION, INDEXER_VELO);
    pros::delay(2250);
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