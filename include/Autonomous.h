#pragma once
#include "main.h"

class Autonomous
{
private:
    /* data */
public:
    Autonomous(ControllerBase& controller, pros::MotorGroup& leftDrive, pros::MotorGroup& rightDrive, pros::MotorGroup& intake, pros::ADIDigitalOut& pistonA, pros::ADIDigitalOut& pistonB, pros::Optical& optical);
    void run();
};