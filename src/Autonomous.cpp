
/**
 * @brief The Autonomous class controls the autonomous behavior of the robot.
 * 
 * This is pretty much an exact copy of the driver control code, but
 * the controller is not a real controller, but a virtual one that gives
 * pre-recorded values at preset times.
 */
#include "Autonomous.h"
#include "DummyController.h"
#include "Ticker.h"


Ticker autonTicker(20);
int decreasedSensitivityFactor = 3;

Autonomous::Autonomous(DummyController& controller, pros::MotorGroup& leftDrive, pros::MotorGroup& rightDrive, pros::MotorGroup& intake, pros::ADIDigitalOut& pistonA, pros::ADIDigitalOut& pistonB, pros::Optical& optical) 
: master(controller), leftDrive(leftDrive), rightDrive(rightDrive), intake(intake), pistonA(pistonA), pistonB(pistonB), optical(optical) {
    // Initialize the autonomous class
}

void Autonomous::run() {



	while (true)
	{
		autonTicker.startTick();
        master.runUpdate();
		
		// Debug
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
						 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
						 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		// Drive
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);
		double rawVoltageOutputLeft = pow(left, decreasedSensitivityFactor) / pow(127, (decreasedSensitivityFactor - 1));
		double rawVoltageOutputRight = pow(right, decreasedSensitivityFactor) / pow(127, (decreasedSensitivityFactor - 1));

		leftDrive.move_velocity(rawVoltageOutputLeft / 127 * 600);
		rightDrive.move_velocity(rawVoltageOutputRight / 127 * 600);

		// Intake
		if (master.get_digital(DIGITAL_L1))
		{
			// Velocity depends on gearset 
			intake.move_velocity(600);
		}
		else if (master.get_digital(DIGITAL_L2))
		{
			intake.move_velocity(-600);
		}
		else
		{
			intake.move_velocity(0);
		}

		// MOGO Mechanism
		if (master.get_digital(DIGITAL_R1))
		{
			// recorder.listenerMOGO(true);
			pistonA.set_value(true);
			pistonB.set_value(true);
		}
		else if (master.get_digital(DIGITAL_R2))
		{
			// recorder.listenerMOGO(false);
			pistonA.set_value(false);
			pistonB.set_value(false);
		}

		if (master.get_digital(DIGITAL_UP))
		{
			break;
		}

		autonTicker.waitTick();
	}

	// Optical
}




