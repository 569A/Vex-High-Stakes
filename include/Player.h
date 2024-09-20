#pragma once
#include "main.h"
#include "Recorder.h"
#include "Recording.h"

class Player
{
private:
    int32_t nextAnalogLeftY = 0;
    int32_t nextAnalogRightY = 0;
    int32_t nextDigitalL1 = 0;
    int32_t nextDigitalL2 = 0;
    int32_t nextDigitalR1 = 0;
    int32_t nextDigitalR2 = 0;

    Recording& recording;
    int index = 0;
public:

    Player(Recording& recording) :
    recording(recording) {
    };


    void runUpdate() {
        if (index == 0) {
            recording.deserializeFromFile("Rerun.569A");
        }
        // Run the player update loop
        // Get the next values from the recording
        nextAnalogLeftY = recording.getAnalogLeftYValue(index);
        nextAnalogRightY = recording.getAnalogRightYValue(index);
        nextDigitalL1 = recording.getDigitalL1Value(index);
        nextDigitalL2 = recording.getDigitalL2Value(index);
        nextDigitalR1 = recording.getDigitalR1Value(index);
        nextDigitalR2 = recording.getDigitalR2Value(index);
        index++;

        
    }
    int32_t get_analog(pros::controller_analog_e_t channel) {
        switch (channel)
        {
        case ANALOG_LEFT_Y:
            return nextAnalogLeftY;
        case ANALOG_RIGHT_Y:
            return nextAnalogRightY;
        default:
            return 0;
        }
    }
    int32_t get_digital(pros::controller_digital_e_t button) {
        switch (button)
        {
        case DIGITAL_L1:
            return nextDigitalL1;
        case DIGITAL_L2:
            return nextDigitalL2;
        case DIGITAL_R1:
            return nextDigitalR1;
        case DIGITAL_R2:
            return nextDigitalR2;
        default:
            return 0;
        }
    }
};
