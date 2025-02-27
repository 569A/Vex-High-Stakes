#pragma once

#include "pros/imu.hpp"

/**
    The inertial sensor can be made more accurate by filtering the data it returns.
    For example, the inertial sensor might be off a consistent 1 degree every 360 degrees.
    We can scale the data it returns to account for this.

 */
class FilteredInertial : public pros::Imu {

private:
    double rotation_scalar; // Multiplier to scale the rotation by (account for consistent error)
    double compensation_factor; // Hooks vibrating makes IMU drift, so we just negate it by adding/subtracting how much it drifts by every 10 ms
public:
    FilteredInertial(std::uint8_t port, double rotation_scalar, double compensation_factor) : pros::Imu(port) {
        this->rotation_scalar = rotation_scalar;
        this->compensation_factor = compensation_factor;
    }

    double get_rotation() const override;
    void update_compensation_factor();
};
