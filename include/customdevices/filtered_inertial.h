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

    pros::Imu imu2; // Second inertial to average with (cancel out each others' drift)

    public:
    FilteredInertial(std::uint8_t port1, std::uint8_t port2, double rotation_scalar, double compensation_factor) 
        : pros::Imu(port1), imu2(port2) {
        this->rotation_scalar = rotation_scalar;
        this->compensation_factor = compensation_factor;
    }

    double get_rotation() const override;
    void update_compensation_factor();
};
