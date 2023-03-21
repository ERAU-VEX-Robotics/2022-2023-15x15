#include "main.h"

/**
 * The implementation of the Drivetrain class
 * This file contains the source code for the Drivetrain class, along with
 * explanations of how each function works
 */

Drivetrain::Drivetrain(std::initializer_list<int> leftPorts,
                       std::initializer_list<int> rightPorts,
                       std::initializer_list<bool> leftRevs,
                       std::initializer_list<bool> rightRevs,
                       pros::motor_gearset_e_t gearset, double wD, double bW,
                       double Pconst, double Iconst, double Dconst)
    : left_motors(leftPorts, leftRevs), right_motors(rightPorts, rightRevs) {

    left_motors.set_gearing(gearset);
    right_motors.set_gearing(gearset);

    wheel_diameter = wD;
    base_width = bW;
    kP = Pconst;
    kI = Iconst;
    kD = Dconst;
}

void Drivetrain::tank_driver(pros::controller_id_e_t controller) {
    /**
     * The driver function uses the okapi controller object to get the values of
     * the Y axes on each controller joystick. Then, each base motor group is
     * set to the value of its corresponding joystick. Since the getAnalog
     * function returns a value between -1 and 1, the controllerSet function is
     * used to set the motors, as it accepts values in that range
     */

    left_motors.move(pros::c::controller_get_analog(controller, ANALOG_LEFT_Y));

    right_motors.move(
        pros::c::controller_get_analog(controller, ANALOG_RIGHT_Y));
}

void Drivetrain::arcade_driver(pros::controller_id_e_t controller) {
    int power = pros::c::controller_get_analog(controller, ANALOG_RIGHT_Y);
    int turn = pros::c::controller_get_analog(controller, ANALOG_RIGHT_X);

    left_motors.move(power + turn);
    right_motors.move(power - turn);
}

void Drivetrain::drivePID(double leftT, double rightT) {
    /**
     * Convert left_targ and right_targ from inches to travel to degrees for the
     * wheels to rotate
     * The distance the wheel travels (if it isn't slipping) over 1 full
     * rotation is equal to its circumference. So, for a wheel with a diameter
     * of 4 inches, it would travel 4*pi inches over 1 full rotation. So, the
     * conversion factor between inches to travel and degrees to rotate is 360
     * degrees/(wheel diameter * pi), as wheel diameter times pi is the inches
     * traveled over 1 rotation, while 360 degrees is degrees rotated over 1
     * rotation
     */
    short int count = 0;
    double left_targ = leftT * 360 / (wheel_diameter * 3.1415) * 2;
    double right_targ = rightT * 360 / (wheel_diameter * 3.1415) * 2;
    // Reset the encoders of the first motor on each side
    left_motors.reset_positions();
    right_motors.reset_positions();
    // Declare or initialize all variables used in the loop
    double left_error = left_targ - left_motors.get_avg_position();
    double right_error = right_targ - right_motors.get_avg_position();
    double left_output;
    double right_output;
    double volt_cap = 0.0;
    // Integral variables are initiated so that the += operator can be used
    // throughout the while loop
    double left_integral = 0;
    double right_integral = 0;
    double left_derivative;
    double right_derivative;
    // Declaring the Previous Error Variable
    double leftPrevError;
    double rightPrevError;
    // Enter a while loop that runs until both sides are within 10 degrees of
    // target rotation
    while (abs(left_error) > 5 || abs(right_error) > 5) {
        printf("\nLeft Targ: %f, Left Error: %f", left_targ, left_error);
        printf("\nRight Targ: %f, Right Error: %f", right_targ, right_error);
        // Calculate the integral
        left_integral += left_error;
        right_integral += right_error;

        // Calculate the derivative
        left_derivative = left_error - leftPrevError;
        right_derivative = right_error - rightPrevError;

        // Set the previous error
        leftPrevError = left_error;
        rightPrevError = right_error;

        // Set the output values
        left_output =
            (left_error * kP) + (left_integral * kI) + (left_derivative * kD);
        right_output = (right_error * kP) + (right_integral * kI) +
                       (right_derivative * kD);

        if (volt_cap < 12000)
            volt_cap += 600;
        else
            volt_cap = 12000;

        if (abs(left_output) > volt_cap)
            left_output = copysign(volt_cap, left_output);
        if (abs(right_output) > volt_cap)
            right_output = copysign(volt_cap, right_output);
        printf("\nLeft Output: %f Right Output: %f", left_output, right_output);

        // Set the motor group voltages to the output velocity levels
        set_voltage(left_output, right_output);
        // Calculate the new error
        left_error = left_targ - left_motors.get_avg_position();
        right_error = right_targ - right_motors.get_avg_position();

        if (left_error == leftPrevError && right_error == rightPrevError)
            count++;
        else
            count = 0;
        if (count >= 5)
            break;
        pros::delay(20);
    }
    set_velocity(0, 0);
    pros::delay(200);
}

void Drivetrain::set_velocity(int left_velo, int right_velo) {
    left_motors.move_velocity(left_velo);
    right_motors.move_voltage(right_velo);
}

void Drivetrain::set_voltage(int left_volt, int right_volt) {
    left_motors.move_voltage(left_volt);
    right_motors.move_voltage(right_volt);
}

void Drivetrain::move_straight(double distance) {
    /**
     * The moveStraight function just slightly simplifies the drivePID
     * function, cutting down on a paramter. This is purely added for
     * convenience/readability. Although not having this function wouldn't
     * change anything signifigant, I wanted to keep the drivePID function
     * private, as, in my mind, it makes sense for an object's PID controller
     * to be kept private.
     */
    drivePID(distance, distance);
}

void Drivetrain::turn_angle(double angle) {
    /**
     * The distance each side needs to rotate can be found with the
     * arc length equation s = r * theta, where theta is the angle
     * in radians, r is the radius of the circle, and s is the
     * arc length.
     * This equation works because the robot turns around a point,
     * so while it might not be perfectly accurate, it is more than a
     * good enough approximation.
     * Relating the equation with the statement below, base_width is r * 2, so
     * we divide the base_width by 2, angle * (3.1415/180) is theta
     * (as the angle passed in is degrees, and the arc length equation
     * uses radians), and turnLength is s
     */
    double turnLength = angle * (3.1415 / 180) * (base_width / 2);
    /**
     * As stated in Drivetrain.hpp, a positive angle will make the robot
     * turn clockwise. To achieve this, the left side of the base must
     * move forward. So, the target for the right side is negative by
     * default. If the input angle is negative, the signs are switched,
     * so the right side goes forward, and the left goes backward, turning
     * the robot counterclockwise
     */
    drivePID(turnLength, -turnLength);
}
