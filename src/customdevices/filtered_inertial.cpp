#include "customdevices/filtered_inertial.h"

double FilteredInertial::get_rotation() const {
    return pros::Imu::get_rotation() * rotation_scalar;
}

void FilteredInertial::update_compensation_factor() {
    //set_rotation(get_rotation() + compensation_factor);
}