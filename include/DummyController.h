#ifndef DUMMY_CONTROLLER_H
#define DUMMY_CONTROLLER_H
#include "main.h"
#include "ControllerBase.h"
#include "Player.h"


class DummyController : public ControllerBase {
private:
    int twentyMsecIter = 0;
    // # of times has been iterated at 20 msec intervals.

    Player* player;



    bool initialized = false;
    bool isPlaying = true;
public:
    DummyController(pros::controller_id_e_t id, bool isPlaying, Player* player) : ControllerBase(id) {
        this->isPlaying = isPlaying;
        this->player = player;
    }

    int32_t get_analog(pros::controller_analog_e_t channel)
    {
        if (isPlaying)
        {
            return player->get_analog(channel);
        }
        return 0;
    }

    int32_t get_digital(pros::controller_digital_e_t button) {
        if (isPlaying) {
            return player->get_digital(button);
        }
        return 0;
    }

    void setButtonPressed(bool pressed) {
        // Simulate a button press event
        if (pressed) {
            std::cout << "Button pressed" << std::endl;
        } else {
            std::cout << "Button released" << std::endl;
        }
    }

    // Override runUpdate method
    void runUpdate() override {
        player->runUpdate();
    }
};

#endif // DUMMY_CONTROLLER_H