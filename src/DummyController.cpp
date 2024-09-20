#include "DummyController.h"


    DummyController::DummyController(pros::controller_id_e_t id, bool isPlaying, Player* player) : ControllerBase(id) {
        this->isPlaying = isPlaying;
        this->player = player;
    }

    int32_t DummyController::get_analog(pros::controller_analog_e_t channel)
    {
        if (isPlaying)
        {
            return player->get_analog(channel);
        }
        return 0;
    }

    int32_t DummyController::get_digital(pros::controller_digital_e_t button) {
        if (isPlaying) {
            return player->get_digital(button);
        }
        return 0;
    }

    void DummyController::setButtonPressed(bool pressed) {
        // Simulate a button press event
        if (pressed) {
            std::cout << "Button pressed" << std::endl;
        } else {
            std::cout << "Button released" << std::endl;
        }
    }

    // Override runUpdate method
    void DummyController::runUpdate() {
        player->runUpdate();
    }
