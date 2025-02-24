#include "auton.h"
#include "lemlib/chassis/trackingWheel.hpp"

Auton::Auton(pros::MotorGroup* left_motor_group, pros::MotorGroup* right_motor_group, pros::Motor* flex_wheel_intake, pros::Motor* hook_intake, pros::adi::DigitalOut* mogo_piston, pros::Motor* arm_motor, pros::Rotation* horizontal_encoder, pros::Rotation* vertical_encoder, pros::Optical* optical, FilteredInertial* imu, pros::Distance* distance) {
    this->left_motor_group = left_motor_group;
    this->right_motor_group = right_motor_group;
    this->flex_wheel_intake = flex_wheel_intake;
    this->hook_intake = hook_intake;
    this->mogo_piston = mogo_piston;
    this->arm_motor = arm_motor;
    this->horizontal_encoder = horizontal_encoder;
    this->vertical_encoder = vertical_encoder;
    this->optical = optical;
    this->imu = imu;
    this->distance = distance;

    this->drivetrain = new lemlib::Drivetrain(left_motor_group, right_motor_group, 12.625, lemlib::Omniwheel::NEW_325, 480, 8);
    
    lemlib::TrackingWheel horizontal_tracking_wheel(horizontal_encoder, 2, -4.25);
    lemlib::TrackingWheel vertical_tracking_wheel(vertical_encoder, 2, 0);
    
    this->sensors = new lemlib::OdomSensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, imu);
    this->lateral_controller = new lemlib::ControllerSettings(5.875, 0, 82, 0, 1, 40, 3, 400, 2.5);
    this->angular_controller = new lemlib::ControllerSettings(2.885, 0, 28, 0, 1.3, 50, 4, 205, 0);
    this->chassis = new lemlib::Chassis(*drivetrain, *lateral_controller, *angular_controller, *sensors, nullptr, nullptr);

}

void Auton::red_neg() {

}

void Auton::red_pos() {

}

void Auton::blue_neg() {

}

void Auton::blue_pos() {

}

void Auton::skills() {

}