#include "lemlib/chassis/chassis.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "customdevices/filtered_inertial.h"


#pragma once
class Auton {
private:
    pros::MotorGroup *left_motor_group;
    pros::MotorGroup *right_motor_group;
    pros::Motor *flex_wheel_intake;
    pros::Motor *hook_intake;
    pros::adi::DigitalOut *mogo_piston;
    pros::Motor *arm_motor;
    pros::Rotation *horizontal_encoder;
    pros::Rotation *vertical_encoder;
    pros::Optical *optical;
    FilteredInertial *imu;
    pros::Distance *distance;

    lemlib::Drivetrain *drivetrain;
    lemlib::OdomSensors *sensors;
    lemlib::ControllerSettings *lateral_controller;
    lemlib::ControllerSettings *angular_controller;
    lemlib::Chassis *chassis;


public:
    Auton(pros::MotorGroup* left_motor_group, pros::MotorGroup* right_motor_group, pros::Motor* flex_wheel_intake, pros::Motor* hook_intake, pros::adi::DigitalOut* mogo_piston, pros::Motor* arm_motor, pros::Rotation* horizontal_encoder, pros::Rotation* vertical_encoder, pros::Optical* optical, FilteredInertial* imu, pros::Distance* distance);
    void red_neg();
    void red_pos();
    void blue_neg();
    void blue_pos();

    void skills();
};