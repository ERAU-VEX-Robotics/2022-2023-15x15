#include "utils.h"

double pid(double kP, double kI, double kD, double error, double *integral,
           double *prev_error) {
    *integral += error;
    double derivative = error - *prev_error;
    *prev_error = error;

    return error * kP + *integral * kI + derivative * kD;
}