#pragma once
#include "main.h"
#include "ControllerBase.h"
#include "Ticker.h"
#include "Recording.h"
#include "Recorder.h"
#include "SlewRateController.h"
#include "AutonSelector.h"

class DriverControl
{
private:
    AutonSelector &autonSelector;

    Ticker driveTicker = Ticker(20);
    bool recordingInput = false;
    Recording recording = Recording(autonSelector);
    Recorder recorder = Recorder(master, recording);
    int decreasedSensitivityFactor = 3;
    int velocity = 600;
    SlewRateController leftDriveSlewRateController = SlewRateController(60);
    SlewRateController rightDriveSlewRateController = SlewRateController(60);
    bool manual = false;
    bool intakeAvailable = true;
    bool competitionMode = false;
    int stage = 1;
    int ticksSinceIntakeCandidateDetected = 0;
    int ticksPerIntakeCheck = 4;
    bool intakeCandidateDetected = false;

    ControllerBase &master;
    okapi::MotorGroup &leftMotors;
    okapi::MotorGroup &rightMotors;
    pros::Motor &intake;
    pros::ADIDigitalOut &pistonA;
    pros::ADIDigitalOut &pistonB;
    pros::Optical &optical;
    pros::Distance &distanceSensor;
public:
    DriverControl(ControllerBase &controller, okapi::MotorGroup &leftMotors, okapi::MotorGroup &rightMotors, pros::Motor &intake, pros::ADIDigitalOut &pistonA, pros::ADIDigitalOut &pistonB, pros::Optical &optical, pros::Distance &distanceSensor, AutonSelector &autonSelector);
    void run();
    void setCompetitonMode(bool mode) {
        competitionMode = mode;
    }
};