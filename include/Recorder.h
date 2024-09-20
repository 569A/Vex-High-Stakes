#include "main.h"
#include "ControllerBase.h"
#include <vector>
#include <fstream>
#include "Recording.h"
#pragma once


using namespace std;

/**
 * @brief The Recorder class is responsible for recording the state of the robot
 * at a given time.
 * 
 * Recording the individual motorgroups and pneumatics would be extremely complex.
 * Instead, we will only record the controller inputs.
 * By running code for driving the robot, then spoofing the controller inputs, we can
 * play back an autonomous program that does everything a driver can.
 * 
 * @author Thomas Hao
 */
class Recorder {
private:
    ControllerBase &controller;
    Recording& recording;

    // pros::MotorGroup &leftDrive;
    // pros::MotorGroup &rightDrive;
    // pros::MotorGroup &intake;
    // pros::ADIDigitalOut &pistonA;
    // pros::ADIDigitalOut &pistonB;

    bool pistonAState;
    bool pistonBState;

    // pros::Optical &optical;

    vector<int> timestamps;
    // vector<int> leftDriveVelocities;
    // vector<int> rightDriveVelocities;
    // vector<int> intakeVelocities;
    // vector<int> pistonAStates;
    // vector<int> pistonBStates;
    // vector<int> opticalValues;

    vector<int> analogLeftYValues;
    vector<int> analogRightYValues;

    vector<int> digitalL1Values;
    vector<int> digitalL2Values;

    vector<int> digitalR1Values;
    vector<int> digitalR2Values;

public:
    
    // Recorder(ControllerBase& controller, pros::MotorGroup& leftDrive, pros::MotorGroup& rightDrive, pros::MotorGroup& intake, pros::ADIDigitalOut& pistonA, pros::ADIDigitalOut& pistonB, pros::Optical& optical) :
    // controller(controller), leftDrive(leftDrive), rightDrive(rightDrive), intake(intake), pistonA(pistonA), pistonB(pistonB), optical(optical)
    // {
    //     // Initialize the recorder
    //     pistonAState = false;
    //     pistonBState = false;

    // }
    Recorder(ControllerBase& controller, Recording& recording) : controller(controller), recording(recording) {
        // Initialize the recorder
        pistonAState = false;
        pistonBState = false;
    }

    void recordUpdate() {
        int time = 0;
        // Record the current state of the robot
            timestamps.push_back(time);

            recording.addAnalogLeftYValue(controller.get_analog(ANALOG_LEFT_Y));
            recording.addAnalogRightYValue(controller.get_analog(ANALOG_RIGHT_Y));

            recording.addDigitalL1Value(controller.get_digital(DIGITAL_L1));
            recording.addDigitalL2Value(controller.get_digital(DIGITAL_L2));

            recording.addDigitalR1Value(controller.get_digital(DIGITAL_R1));
            recording.addDigitalR2Value(controller.get_digital(DIGITAL_R2));
            
            // pros::delay(20);
            time += 20;
    }

    void stop() {
        // Stop the recording
    }

    Recording& getRecording() {
        // Return the recording
        return recording;
    }

    void saveRecording() {
        // Save the recording to a file
        recording.serializeToFile("Rerun.569A");
    }
    // void deserializeFromFile(string filename) {
    //     // Deserializes controller history from SD card.
    //     ifstream file(filename);
    //     if (file.is_open()) {
    //         string line;
    //         while (getline(file, line)) {
    //             // Parse and store into vectors
    //             stringstream ss(line);
    //             string token;
    //             vector<int> values;
    //             while (getline(ss, token, ',')) {
    //                 values.push_back(stoi(token));
    //             }
    //             if (values.size() == 7) {
    //                 timestamps.push_back(values[0]);
    //                 analogLeftYValues.push_back(values[1]);
    //                 analogRightYValues.push_back(values[2]);
    //                 digitalL1Values.push_back(values[3]);
    //                 digitalL2Values.push_back(values[4]);
    //                 digitalR1Values.push_back(values[5]);
    //                 digitalR2Values.push_back(values[6]);
    //             }
    //         }
    //         file.close();
    //     } else {
    //         cout << "Unable to open file" << endl;
    //     }
    // }


    // void listenerMOGO(bool newState) {
    //     // Listen for the MOGO mechanism
    //     pistonAState = newState;
    //     pistonBState = newState;
    // }
};
