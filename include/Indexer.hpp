#ifndef INDEXER_HPP
#define INDEXER_HPP

/**
 * \file Indexer.hpp
 * This file contains the class declaration for the Indexer subsystem, which
 * pushes disks into the flywheel from their storage location. Our indexer
 * implementation uses a "puncher". Punchers involve having some object be
 * connected to a motor through the combination of a rack gear and a normal
 * gear. However, the normal gear has had some of its teeth removed, making it
 * into what is called a slip gear. As the gear is rotated by the motor, it
 * pulls the object back because of the rack gear, but once the part of the
 * normal gear that is missing teeth reaches the rack gear, the object is free
 * to move. This object also has rubber bands connected to it that cause it to
 * move toward the disk when nothing is holding it back.
 */

#include "Motor_Group.hpp"
#include "api.h"
#include <initializer_list>

class Indexer {
  private:
    Motor_Group motors;

    int degrees_to_rotate;

  public:
    // Constructor for the Indexer class
    Indexer(std::initializer_list<int> ports, std::initializer_list<bool> revs);

    // Sets the degrees the gear needs to rotate for each time the puncher fires
    void set_rotation(int degrees_to_rotate);

    // Function that controls the indexer in opcontrol
    void driver(pros::controller_id_e_t controller,
                pros::controller_digital_e_t fire_btn);

    // Moves the gear enough to send one disk into the flywheel
    void punch_disk();
};

#endif /* Indexer.hpp */