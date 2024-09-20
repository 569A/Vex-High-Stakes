#pragma once
#include "main.h"
#include "ControllerBase.h" 
#include "DummyController.h"

class Autonomous
{
private:
    DummyController &master;
    pros::MotorGroup &leftDrive;
    pros::MotorGroup &rightDrive;
    pros::MotorGroup &intake;
    pros::ADIDigitalOut &pistonA;
    pros::ADIDigitalOut &pistonB;
    pros::Optical &optical;
public:
    Autonomous(DummyController &controller, pros::MotorGroup &leftDrive, pros::MotorGroup &rightDrive, pros::MotorGroup &intake, pros::ADIDigitalOut &pistonA, pros::ADIDigitalOut &pistonB, pros::Optical &optical);
    void run();
};