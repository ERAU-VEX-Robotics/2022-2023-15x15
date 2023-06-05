#include "Drivetrain.hpp"
#include "stdio.h"
#include <cmath>

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

double Drivetrain::convert_inches_to_degrees(double inches) {
    /**
     * Formula Explanation:
     * We treat inches as an arc length, so we divide it by the tracking wheel
     * radius (since the tracking wheels are the ones from which all
     * measurements are gathered). This is based off of the arc length equation,
     * s = r * theta. In this case, we want theta, the central angle, or the
     * angle that the tracking wheel needs to rotate.
     *
     * Next, we use 180 / pi to convert from radians to degrees, since both the
     * internal motor encoders and ADI encoders return values in degrees.
     *
     * Finally, to account for any gear ratio between the encoder and the
     * tracking wheel (this typically only arises when using the internal motor
     * encoders), we multiply by the gear ratio, defined as (wheel connected to
     * encoder / wheel connected to the wheel). If the ratio is greater than 1,
     * then the gear on the encoder has more teeth than that of the wheel. So,
     * the rotation of the wheel is undercounted. Thus, multiplying by the gear
     * ratio accounts for this undermeasuring.
     */
    return inches / tracking_wheel_radius * 180 / 3.1415 /
           tracking_wheel_gear_ratio;
}

double Drivetrain::arc_len(double angle, double radius) {
    // The arc length formula, including converting the angle from degrees
    return radius * angle * 3.1415 / 180.0;
}

void Drivetrain::add_adi_encoders(uint8_t left_encdr_top_port,
                                  uint8_t left_encdr_bot_port,
                                  bool left_encdr_rev,
                                  uint8_t right_encdr_top_port,
                                  uint8_t right_encdr_bot_port,
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

    int count = 0;

    while (true) {
        if (using_encdrs) {
            left_error = left_targ - pros::c::adi_encoder_get(left_encdr);
            right_error = right_targ - pros::c::adi_encoder_get(right_encdr);
        } else {
            left_error = left_targ - left_motors.get_avg_position();
            right_error = right_targ - right_motors.get_avg_position();
        }

        left_integral += left_error;
        right_integral += right_error;

        if (reset_pid_vars) {
            left_integral = 0;
            left_prev_error = 0;
            left_error = 0;

            right_integral = 0;
            right_prev_error = 0;
            right_error = 0;

            is_settled = false;
            reset_pid_vars = false;
        } else if (fabs(left_error) < settled_threshold &&
                   fabs(right_error) < settled_threshold) {
            is_settled = true;
        } else
            is_settled = false;

        int left_voltage, right_voltage;

        if (left_error == left_prev_error && right_error == right_prev_error) {
            ++count;
        } else
            count = 0;

        if (count > 5) {
            is_settled = true;
            count = 0;
        }

        if (is_settled) {
            left_voltage = 0;
            right_voltage = 0;
            left_motors.brake();
            right_motors.brake();
            pros::delay(5);

            continue;
        } else {
            left_voltage = left_error * kP + left_integral * kI +
                           (left_error - left_prev_error) * kD;
            right_voltage = right_error * kP + right_integral * kI +
                            (right_error - right_prev_error) * kD;
        }

        if (abs(left_voltage) > 12000)
            left_voltage = copysign(12000, left_voltage);
        if (abs(right_voltage) > 12000)
            right_voltage = copysign(12000, right_voltage);
        left_motors.move_voltage(left_voltage);
        right_motors.move_voltage(right_voltage);

#ifdef D_DEBUG
        printf("Left Error: %.2lf\nRight Error: %.2lf\nSettled: %d\n",
               left_error, right_error, is_settled.load());
        print_telemetry(E_MOTOR_GROUP_TELEM_PRINT_VOLTAGE,
                        E_MOTOR_GROUP_TELEM_PRINT_VOLTAGE);
#endif
        left_prev_error = left_error;
        right_prev_error = right_error;
        pros::delay(2);
    }
}

void Drivetrain::set_pid_consts(double Pconst, double Iconst, double Dconst) {
    kP = Pconst;
    kI = Iconst;
    kD = Dconst;
}

void Drivetrain::move_straight(double inches) {
    double temp = convert_inches_to_degrees(inches);

    reset_pid_state(temp, temp);
}

void Drivetrain::turn_angle(double angle) {

    // Convert the angle to turn into degrees for the wheels to rotate
    // This consists of 2 parts. First, we turn the angle into the number of
    // inches each side needs to move. Then, we turn that into degrees
    double temp = convert_inches_to_degrees(arc_len(angle, track_distance));

    reset_pid_state(temp, -temp);
}

void Drivetrain::init_pid_task() {
    pid_task =
        pros::c::task_create(trampoline, this, TASK_PRIORITY_DEFAULT,
                             TASK_STACK_DEPTH_DEFAULT, "Drivetrain PID Task");
}

void Drivetrain::set_velo(int left_velo, int right_velo) {
    left_motors.move_velocity(left_velo);
    right_motors.move_velocity(right_velo);
}

void Drivetrain::pause_pid_task() { pros::c::task_suspend(pid_task); }

void Drivetrain::resume_pid_task() { pros::c::task_resume(pid_task); }

void Drivetrain::end_pid_task() {
    pros::c::task_delete(pid_task);
    left_motors.move(0);
    right_motors.move(0);
}

void Drivetrain::wait_until_settled() {
    pros::delay(200);
    while (!is_settled) {
        pros::delay(2);
    }
    pros::delay(200);
}
void Drivetrain::set_settled_threshold(double threshold) {
    settled_threshold = threshold;
}

void Drivetrain::set_drivetrain_dimensions(double tw, double twr,
                                           double gear_ratio) {
    track_distance = tw / 2;
    tracking_wheel_radius = twr;
    tracking_wheel_gear_ratio = gear_ratio;
}

void Drivetrain::set_voltage_limit(int limit) {
    left_motors.set_voltage_limits(limit);
    right_motors.set_voltage_limits(limit);
}

void Drivetrain::tank_driver(pros::controller_id_e_t controller,
                             pros::controller_digital_e_t rev_btn) {

    if (pros::c::controller_get_digital_new_press(controller, rev_btn))
        rev_control = !rev_control;

    if (rev_control) {
        right_motors.move(-pros::c::controller_get_analog(
            controller, pros::E_CONTROLLER_ANALOG_LEFT_Y));
        left_motors.move(-pros::c::controller_get_analog(
            controller, pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    } else {
        left_motors.move(pros::c::controller_get_analog(
            controller, pros::E_CONTROLLER_ANALOG_LEFT_Y));
        right_motors.move(pros::c::controller_get_analog(
            controller, pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    }
}

void Drivetrain::tank_driver_poly(pros::controller_id_e_t controller,
                                  double pow,
                                  pros::controller_digital_e_t rev_btn) {

    int left, right;

    if (pros::c::controller_get_digital_new_press(controller, rev_btn))
        rev_control = !rev_control;

    if (rev_control) {
        right = -pros::c::controller_get_analog(
            controller, pros::E_CONTROLLER_ANALOG_LEFT_Y);
        left = -pros::c::controller_get_analog(
            controller, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    } else {
        left = pros::c::controller_get_analog(controller,
                                              pros::E_CONTROLLER_ANALOG_LEFT_Y);
        right = pros::c::controller_get_analog(
            controller, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    }
    // We subtract 1 from pow since we multiply by the initial value - thus
    // restoring the original range
    left_motors.move(left * std::pow((std::abs(left) / 127.0), pow - 1));
    right_motors.move(right * std::pow((std::abs(right) / 127.0), pow - 1));
}

void Drivetrain::arcade_driver(pros::controller_id_e_t controller,
                               pros::controller_digital_e_t rev_btn,
                               bool use_right) {
    int power;
    int turn;
    if (use_right) {
        power = pros::c::controller_get_analog(
            controller, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        turn = pros::c::controller_get_analog(
            controller, pros::E_CONTROLLER_ANALOG_RIGHT_X);
    } else {
        power = pros::c::controller_get_analog(
            controller, pros::E_CONTROLLER_ANALOG_LEFT_Y);
        turn = pros::c::controller_get_analog(controller,
                                              pros::E_CONTROLLER_ANALOG_LEFT_X);
    }

    if (pros::c::controller_get_digital_new_press(controller, rev_btn))
        rev_control = !rev_control;

    if (rev_control) {
        left_motors.move(-power + turn);
        right_motors.move(-power - turn);
    } else {
        left_motors.move(power + turn);
        right_motors.move(power - turn);
    }
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

void Drivetrain::reset_pid_state(double new_left_targ, double new_right_targ) {
    pros::c::task_suspend(pid_task);

    reset_pid_vars = true;

    // Reset the encoder positions
    if (using_encdrs) {
        pros::c::adi_encoder_reset(left_encdr);
        pros::c::adi_encoder_reset(right_encdr);
    } else {
        left_motors.reset_positions();
        right_motors.reset_positions();
    }

    // Update targets
    left_targ = new_left_targ;
    right_targ = new_right_targ;

    pros::c::task_resume(pid_task);
}