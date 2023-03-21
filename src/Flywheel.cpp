#include "Flywheel.hpp"
#include <cmath>
#include <cstdio>

Flywheel::Flywheel(std::initializer_list<int> ports,
                   std::initializer_list<bool> reverses)
    : motors(ports, reverses) {
    motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    /**
     * The motor uses a 3D-printed replacement for the gear cartridge to
     * directly drive the axle from the output of the motor, which yields an RPM
     * of 3600 from the motor. However, this is not an option to set the motor's
     * gearing to, so I approximate using the maximum internal motor cartridge,
     * which is 600 RPM. This value is important for the move_velocity function
     * used to run the motors, in case I need to use that function.
     */
    motors.set_gearing(pros::E_MOTOR_GEAR_BLUE);
    motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void Flywheel::task_fn() {
    double error = 0;
    double prev_error = 0;
    double voltage = tbh_estimate;
    double base_voltage = 2 * tbh_estimate - 12000;

    while (true) {
        if (update_tbh_consts) {
            voltage = tbh_estimate;
            base_voltage = 2 * tbh_estimate - 12000;
        }

        error = flywheel_velo - motors.get_avg_velocity();
        voltage += error * tbh_gain;

        if (std::signbit(error) != std::signbit(prev_error))
            voltage = 0.5 * (voltage + base_voltage);

        base_voltage = voltage;

        if (fabs(voltage) > 12000)
            voltage = copysign(12000, voltage);
#ifdef F_DEBUG
        printf("Flywheel error: %.2lf\n", error);
        printf("Flywheel set voltage: %.2lf\n", voltage);
        print_telemetry(E_MOTOR_GROUP_TELEM_PRINT_VOLTAGE |
                        E_MOTOR_GROUP_TELEM_PRINT_CURRENT |
                        E_MOTOR_GROUP_TELEM_PRINT_TEMPERATURE |
                        E_MOTOR_GROUP_TELEM_PRINT_VELOCITY);
#endif
        motors.move_voltage(voltage);
#ifdef F_DEBUG
        pros::delay(200);
#else
        pros::delay(2);
#endif
        prev_error = error;
    }
}

void Flywheel::trampoline(void *param) {
    if (param) {
        Flywheel *that = static_cast<Flywheel *>(param);
        that->task_fn();
    }
}

void Flywheel::init_task() {
    task = pros::c::task_create(trampoline, this, TASK_PRIORITY_DEFAULT,
                                TASK_STACK_DEPTH_DEFAULT, "Flywheel PID Task");
}

void Flywheel::pause_task() {
    pros::c::task_suspend(task);
    stop();
}

void Flywheel::resume_task() { pros::c::task_resume(task); }

void Flywheel::end_task() {
    pause_task();
    pros::c::task_delete(task);
}

void Flywheel::set_target_velo(int velo) { flywheel_velo = velo; }

void Flywheel::set_tbh_consts(double gain, double estimate) {
    tbh_gain = gain;
    tbh_estimate = estimate;
    update_tbh_consts = true;
}

void Flywheel::set_speed_fast() {
    set_target_velo(FLYWHEEL_FAST_TARG);
    set_tbh_consts(FLYWHEEL_TBH_GAIN, FLYWHEEL_FAST_ESTIMATE);
}

void Flywheel::set_speed_slow() {
    set_target_velo(FLYWHEEL_SLOW_TARG);
    set_tbh_consts(FLYWHEEL_TBH_GAIN, FLYWHEEL_SLOW_ESTIMATE);
}

void Flywheel::driver(pros::controller_id_e_t controller,
                      pros::controller_digital_e_t pwr_button,
                      pros::controller_digital_e_t spd_button) {
    static bool running = false;
    static bool slow = false;
    if (pros::c::controller_get_digital_new_press(controller, pwr_button)) {
        running = !running; // Toggle flywheel status
        if (running)
            resume_task();
        else
            pause_task();
    }

    if (pros::c::controller_get_digital_new_press(controller, spd_button)) {
        slow = !slow; // Toggle flywheel status
        if (slow) {
            set_speed_slow();
        } else {
            set_speed_fast();
        }
    }
}

void Flywheel::set_velocity(int16_t velocity) {
    motors.move_velocity(velocity);
}

void Flywheel::stop() { motors.brake(); }

void Flywheel::print_telemetry(uint8_t vals_to_print) {
    printf("Flywheel Telemetry\n");
    motors.print_telemetry(vals_to_print);
}
