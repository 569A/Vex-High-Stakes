
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
bool manual = false;

int ticksSinceIntakeCandidateDetected = 0;
bool intakeCandidateDetected = false;
bool intakeAvailable = true;


Autonomous::Autonomous(DummyController& controller, pros::MotorGroup& leftDrive, pros::MotorGroup& rightDrive, pros::Motor& intake, pros::ADIDigitalOut& pistonA, pros::ADIDigitalOut& pistonB, pros::Optical& optical, pros::Distance& distanceSensor) 
: master(controller), leftDrive(leftDrive), rightDrive(rightDrive), intake(intake), pistonA(pistonA), pistonB(pistonB), optical(optical), distanceSensor(distanceSensor) {
    // Initialize the autonomous class
}

void Autonomous::run() {

	while (true)
	{
		autonTicker.startTick();
        master.runUpdate();
		
		// Debug

		// Drive
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);
		pros::lcd::set_text(0, std::to_string(left));
		double rawVoltageOutputLeft = pow(left, decreasedSensitivityFactor) / pow(127, (decreasedSensitivityFactor - 1));
		double rawVoltageOutputRight = pow(right, decreasedSensitivityFactor) / pow(127, (decreasedSensitivityFactor - 1));

		leftDrive.move_velocity(rawVoltageOutputLeft / 127 * 600);
		rightDrive.move_velocity(rawVoltageOutputRight / 127 * 600);

		// Intake
		if (manual)
		{
			if (master.get_digital(DIGITAL_L1))
			{
				// Velocity depends on gearset 
				intake.move_velocity(600);
			}
			else if (master.get_digital(DIGITAL_L2))
			{
				intake.move_velocity(-600);
			}
			else if (intakeAvailable) // If intake is automatically scoring, don't allow manual control
			{
				intake.move_velocity(0);
			}
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

		if (distanceSensor.get() < 100)
		{
			if (ticksSinceIntakeCandidateDetected > 3) {
				if (intakeAvailable)
				{
					intake.move_relative(-3695, 600);
					intakeAvailable = false;
				} else {
					if (intake.get_position() > intake.get_target_position() - 20 && intake.get_position() < intake.get_target_position() + 20)
					{
						intakeCandidateDetected = false;
						intakeAvailable = true;
					}
				}
			} else {
				if (!intakeCandidateDetected) {
					intakeCandidateDetected = true;
					ticksSinceIntakeCandidateDetected = 0;
				}
				ticksSinceIntakeCandidateDetected++;
			}	
		}
		autonTicker.waitTick();
	}

	// Optical
}




