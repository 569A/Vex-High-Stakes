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
    pros::Motor &intake;
    pros::ADIDigitalOut &pistonA;
    pros::ADIDigitalOut &pistonB;
    pros::Optical &optical;
    pros::Distance &distanceSensor;
public:
    Autonomous(DummyController &controller, pros::MotorGroup &leftDrive, pros::MotorGroup &rightDrive, pros::Motor &intake, pros::ADIDigitalOut &pistonA, pros::ADIDigitalOut &pistonB, pros::Optical &optical, pros::Distance &distanceSensor);
    void run();
};