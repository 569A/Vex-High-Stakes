#include "customdevices/filtered_inertial.h"

double FilteredInertial::get_rotation() const {
    return pros::Imu::get_rotation() * rotation_scalar;
}