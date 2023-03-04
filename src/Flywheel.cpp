#include "Flywheel.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "utils.h"
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

void Flywheel::pid_task_fn() {
    double integral = 0;
    double prev_error = 0;
    double error = 0;

    while (true) {
        error = flywheel_velo - motors.get_avg_velocity();
        double voltage = pid(kP, kI, kD, error, &integral, &prev_error);

        if (fabs(voltage) > 12000)
            voltage = copysign(12000, voltage);
#ifdef F_DEBUG
        printf("Flywheel error: %.2lf\n", error);
        printf("Flywheel set voltage: %.2lf\n", voltage);
        print_telemetry(E_MOTOR_GROUP_TELEM_PRINT_VOLTAGE |
                        E_MOTOR_GROUP_TELEM_PRINT_CURRENT |
                        E_MOTOR_GROUP_TELEM_PRINT_TEMPERATURE);
#endif
        motors.move_voltage(voltage);
#ifdef F_DEBUG
        pros::delay(200);
#else
        pros::delay(2);
#endif
    }
}

void Flywheel::trampoline(void *param) {
    if (param) {
        Flywheel *that = static_cast<Flywheel *>(param);
        that->pid_task_fn();
    }
}

void Flywheel::init_pid_task() {
    pid_task =
        pros::c::task_create(trampoline, this, TASK_PRIORITY_DEFAULT,
                             TASK_STACK_DEPTH_DEFAULT, "Flywheel PID Task");
}

void Flywheel::pause_pid_task() {
    pros::c::task_suspend(pid_task);
    stop();
}

void Flywheel::resume_pid_task() { pros::c::task_resume(pid_task); }

void Flywheel::end_pid_task() {
    pause_pid_task();
    pros::c::task_delete(pid_task);
}

void Flywheel::set_target_velo(int velo) { flywheel_velo = velo; }

void Flywheel::set_pid_consts(double Pconst, double Iconst, double Dconst) {
    kP = Pconst;
    kI = Iconst;
    kD = Dconst;
}

void Flywheel::driver(pros::controller_id_e_t controller,
                      pros::controller_digital_e_t pwr_button,
                      pros::controller_digital_e_t rev_button) {
    static bool running = false;
    static bool reversed = false;
    if (pros::c::controller_get_digital_new_press(controller, pwr_button)) {
        running = !running; // Toggle flywheel status
        if (running)
            resume_pid_task();
        else
            pause_pid_task();
    }

    if (pros::c::controller_get_digital_new_press(controller, rev_button)) {
        reversed = !reversed; // Toggle flywheel status
        if (reversed)
            set_target_velo(-FLYWHEEL_REV_TARG);
        else
            set_target_velo(FLYWHEEL_FWD_TARG);
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
