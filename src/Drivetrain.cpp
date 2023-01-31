#include "Drivetrain.hpp"
#include "pros/adi.h"
#include "pros/rtos.hpp"
#include "utils.h"

Drivetrain::Drivetrain(std::initializer_list<int> left_ports,
                       std::initializer_list<int> right_ports,
                       std::initializer_list<bool> left_revs,
                       std::initializer_list<bool> right_revs)
    : left_motors(left_ports, left_revs),
      right_motors(right_ports, right_revs) {
    left_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    right_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}

void Drivetrain::trampoline(void *param) {
    if (param) {
        Drivetrain *that = static_cast<Drivetrain *>(param);
        that->pid_task_fn();
    }
}

void Drivetrain::add_adi_encoders(char left_encdr_top_port,
                                  char left_encdr_bot_port, bool left_encdr_rev,
                                  char right_encdr_top_port,
                                  char right_encdr_bot_port,
                                  bool right_encdr_rev) {
    using_encdrs = true;
    left_encdr = pros::c::adi_encoder_init(left_encdr_top_port,
                                           left_encdr_bot_port, left_encdr_rev);

    right_encdr = pros::c::adi_encoder_init(
        right_encdr_top_port, right_encdr_bot_port, right_encdr_rev);
}

void Drivetrain::pid_task_fn() {
    double left_integral = 0;
    double left_prev_error = 0;
    double left_error = 0;

    double right_integral = 0;
    double right_prev_error = 0;
    double right_error = 0;

    while (true) {
        if (using_encdrs) {
            left_error = left_targ - pros::c::adi_encoder_get(left_encdr);
            right_error = right_targ - pros::c::adi_encoder_get(right_encdr);
        } else {
            left_error = left_targ - left_motors.get_avg_position();
            right_error = right_targ - right_motors.get_avg_position();
        }

        if (fabs(left_error) < settled_threshold &&
            fabs(right_error) < settled_threshold)
            is_settled = true;
        else
            is_settled = false;

        int left_voltage, right_voltage;

        if (use_turn_consts) {
            left_voltage = pid(kP_turn, kI_turn, kD_turn, left_error,
                               &left_integral, &left_prev_error);
            right_voltage = pid(kP_turn, kI_turn, kD_turn, right_error,
                                &right_integral, &right_prev_error);
        } else {
            left_voltage = pid(kP_straight, kI_straight, kD_straight,
                               left_error, &left_integral, &left_prev_error);
            right_voltage =
                pid(kP_straight, kI_straight, kD_straight, right_error,
                    &right_integral, &right_prev_error);
        }

        if (abs(left_voltage) > 12000)
            left_voltage = copysign(12000, left_voltage);
        if (abs(right_voltage) > 12000)
            right_voltage = copysign(12000, right_voltage);
#ifdef DEBUG
        printf("Left Error: %.2lf\nRight Error: %.2lf\n", left_error,
               right_error);
        print_telemetry(E_MOTOR_GROUP_TELEM_PRINT_VOLTAGE,
                        E_MOTOR_GROUP_TELEM_PRINT_VOLTAGE |
                            E_MOTOR_GROUP_TELEM_PRINT_POSITION);

#endif

        left_motors.move_voltage(left_voltage);
        right_motors.move_voltage(right_voltage);

        pros::delay(2);
    }
}

void Drivetrain::set_pid_straight_consts(double Pconst, double Iconst,
                                         double Dconst) {
    kP_straight = Pconst;
    kI_straight = Iconst;
    kD_straight = Dconst;
}

void Drivetrain::set_pid_turn_consts(double Pconst, double Iconst,
                                     double Dconst) {
    kP_turn = Pconst;
    kI_turn = Iconst;
    kD_turn = Dconst;
}

void Drivetrain::move_straight(double inches) {
    // Convert inches to degrees for the wheels to rotate
    double temp =
        inches / tracking_wheel_radius * 180 / M_PI * tracking_wheel_gear_ratio;

    left_targ = temp;
    right_targ = temp;
    // Set the encoder positions to 0
    if (using_encdrs) {
        pros::c::adi_encoder_reset(left_encdr);
        pros::c::adi_encoder_reset(right_encdr);
    } else {
        left_motors.reset_positions();
        right_motors.reset_positions();
    }
    is_settled = false;
}
void Drivetrain::turn_angle(double angle) {
    if (using_encdrs) {
        pros::c::adi_encoder_reset(left_encdr);
        pros::c::adi_encoder_reset(right_encdr);
    } else {
        left_motors.reset_positions();
        right_motors.reset_positions();
    }
    // Convert the angle to turn into degrees for the wheels to rotate
    // This consists of 2 parts. First, we turn the angle into the number of
    // inches each side needs to move. Then, we turn that into degrees
    double temp = (angle * (M_PI / 180) * track_radius) /
                  tracking_wheel_radius * 180 / M_PI *
                  tracking_wheel_gear_ratio;

    printf("Target: %.2lf\n", temp);

    left_targ = temp;
    right_targ = -temp;
    // Set the encoder positions to 0

    is_settled = false;
}

void Drivetrain::init_pid_task() {
    pid_task =
        pros::c::task_create(trampoline, this, TASK_PRIORITY_DEFAULT,
                             TASK_STACK_DEPTH_DEFAULT, "Drivetrain PID Task");
}

void Drivetrain::end_pid_task() {
    pros::c::task_delete(pid_task);
    left_motors.move(0);
    right_motors.move(0);
}

void Drivetrain::wait_until_settled() {
    while (!is_settled)
        pros::delay(20);
}
void Drivetrain::set_settled_threshold(double threshold) {
    settled_threshold = threshold;
}

void Drivetrain::set_drivetrain_dimensions(double tw, double twr,
                                           double gear_ratio) {
    track_radius = tw / 2;
    tracking_wheel_radius = twr;
    tracking_wheel_gear_ratio = gear_ratio;
}

void Drivetrain::tank_driver(pros::controller_id_e_t controller) {
    left_motors.move(pros::c::controller_get_analog(
        controller, pros::E_CONTROLLER_ANALOG_LEFT_Y));
    right_motors.move(pros::c::controller_get_analog(
        controller, pros::E_CONTROLLER_ANALOG_RIGHT_Y));
}

void Drivetrain::arcade_driver(pros::controller_id_e_t controller) {
    int power = pros::c::controller_get_analog(
        controller, pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = pros::c::controller_get_analog(controller,
                                              pros::E_CONTROLLER_ANALOG_LEFT_X);

    left_motors.move(power + turn);
    right_motors.move(power - turn);
}

void Drivetrain::print_telemetry(uint8_t left_vals, uint8_t right_vals) {
    if (left_vals) {
        printf("Left Motor Telemetry\n");
        left_motors.print_telemetry(left_vals);
    }
    printf("\n");
    if (right_vals) {
        printf("Right Motor Telemetry\n");
        right_motors.print_telemetry(right_vals);
    }
    printf("\n");
    printf("\n");
}
