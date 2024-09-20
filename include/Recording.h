#pragma once
#include <string>
#include <vector>

class Recording {
private:
    std::vector<int> analogLeftYValues;
    std::vector<int> analogRightYValues;

    std::vector<int> digitalL1Values;
    std::vector<int> digitalL2Values;

    std::vector<int> digitalR1Values;
    std::vector<int> digitalR2Values;
public:
    Recording() {
    }

    void serializeToFile(std::string filename);
    void deserializeFromFile(std::string filename);

    void addAnalogLeftYValue(int value) {
        analogLeftYValues.push_back(value);
    }

    void addAnalogRightYValue(int value) {
        analogRightYValues.push_back(value);
    }

    void addDigitalL1Value(int value) {
        digitalL1Values.push_back(value);
    }

    void addDigitalL2Value(int value) {
        digitalL2Values.push_back(value);
    }

    void addDigitalR1Value(int value) {
        digitalR1Values.push_back(value);
    }

    void addDigitalR2Value(int value) {
        digitalR2Values.push_back(value);
    }


    //Getters
    int getAnalogLeftYValue(int index) {
        return analogLeftYValues[index];
    }

    int getAnalogRightYValue(int index) {
        return analogRightYValues[index];
    }

    int getDigitalL1Value(int index) {
        return digitalL1Values[index];
    }

    int getDigitalL2Value(int index) {
        return digitalL2Values[index];
    }

    int getDigitalR1Value(int index) {
        return digitalR1Values[index];
    }

    int getDigitalR2Value(int index) {
        return digitalR2Values[index];
    }

    

};