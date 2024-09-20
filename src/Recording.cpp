
/**
 * @brief Class for storing controller inputs.
 * 
 * This class provides functionality to write / read controller inputs to/from a file.
 * It then stores them as variables.
 */
#include "Recording.h"
#include <iostream>
#include <fstream>
#include <sstream>


void Recording::serializeToFile(std::string filename) {
    // Serialize the recording to a file
    FILE* file = fopen(filename.c_str(), "w");
    if (file != nullptr) {
        for (int i = 0; i < analogLeftYValues.size(); i++) {
            fprintf(file, "%d,", analogLeftYValues[i]);
            fprintf(file, "%d,", analogRightYValues[i]);
            fprintf(file, "%d,", digitalL1Values[i]);
            fprintf(file, "%d,", digitalL2Values[i]);
            fprintf(file, "%d,", digitalR1Values[i]);
            fprintf(file, "%d\n", digitalR2Values[i]);
        }
        fclose(file);
    } else {
        std::cout << "Unable to open file" << std::endl;
    }
}

void Recording::deserializeFromFile(const std::string filename) {
    FILE* file = fopen(("/usd/" + filename).c_str(), "r");
    if (file == NULL) {
        std::cout << "Unable to open file" << std::endl;
        return;
    }

    char line[256];
    while (fgets(line, sizeof(line), file)) {
        std::istringstream iss(line);
        std::string value;
        while (std::getline(iss, value, ',')) {
            // Assuming the values are integers, you can convert them to the appropriate data types
            int analogLeftYValue = std::stoi(value);
            std::getline(iss, value, ',');
            int analogRightYValue = std::stoi(value);
            std::getline(iss, value, ',');
            int digitalL1Value = std::stoi(value);
            std::getline(iss, value, ',');
            int digitalL2Value = std::stoi(value);
            std::getline(iss, value, ',');
            int digitalR1Value = std::stoi(value);
            std::getline(iss, value);
            int digitalR2Value = std::stoi(value);

            // Store the deserialized values in the respective vectors
            analogLeftYValues.push_back(analogLeftYValue);
            analogRightYValues.push_back(analogRightYValue);
            digitalL1Values.push_back(digitalL1Value);
            digitalL2Values.push_back(digitalL2Value);
            digitalR1Values.push_back(digitalR1Value);
            digitalR2Values.push_back(digitalR2Value);
        }
    }

    fclose(file);
}
