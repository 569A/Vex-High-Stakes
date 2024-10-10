
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
    // /usd/ is needed for PROS to write to the SD card
    FILE* file = fopen(("/usd/" + filename).c_str(), "w");

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

    fclose(file);
}
