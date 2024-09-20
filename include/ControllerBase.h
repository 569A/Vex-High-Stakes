#include "main.h"

#pragma once

class ControllerBase : public pros::Controller {
public:
    
    ControllerBase(pros::controller_id_e_t id) : pros::Controller(id) {
        // Initialize the controller
    }

    virtual void runUpdate() {
        // Run the controller update loop
    }
    
    // Add your custom methods here
    
};
