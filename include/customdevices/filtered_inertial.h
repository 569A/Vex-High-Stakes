#pragma once

#include "pros/imu.hpp"

/**
    The inertial sensor can be made more accurate by filtering the data it returns.
    For example, the inertial sensor might be off a consistent 1 degree every 360 degrees.
    We can scale the data it returns to account for this.

 */
class FilteredInertial : public pros::Imu {

private:
    double rotation_scalar;
public:
    FilteredInertial(std::uint8_t port, double rotation_scalar) : pros::Imu(port) {
        this->rotation_scalar = rotation_scalar;
    }

    double get_rotation() const override;
};
