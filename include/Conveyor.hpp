#ifndef CONVEYOR_HPP
#define CONVEYOR_HPP

#include "Motor_Group.hpp"
#include "pros/misc.h"
#include <initializer_list>

/**
 * \file Conveyor.hpp
 * This file contains the class declaration for the Conveyor class, which
 * represents the physical conveyor that loads the flywheel.
 */

class Conveyor {
  private:
    Motor_Group motors;

  public:
    Conveyor(std::initializer_list<int> ports,
             std::initializer_list<bool> revs);

    void forward();

    void reverse();

    void stop();

    void rotate(double degrees);

    void driver(pros::controller_id_e_t controller,
                pros::controller_digital_e_t fwd_btn,
                pros::controller_digital_e_t rev_btn);
};

#endif /* Conveyor.hpp */