#include "main.h"
#include "DummyController.h"
#include "Autonomous.h"
#include "Recording.h"
#include "Ticker.h"
#include <cmath>

using namespace std;

string colorMode = "blue";

	
// Controller
ControllerBase master(pros::E_CONTROLLER_MASTER);
ControllerBase& masterRef = master;


// Motor groups for drive
pros::MotorGroup leftMotors({-11, 12, -13});
pros::MotorGroup& leftMotorsRef = leftMotors;

pros::MotorGroup rightMotors({18, -19, 20});
pros::MotorGroup& rightMotorsRef = rightMotors;

// Intake
pros::Motor intake(10);
pros::Motor& intakeRef = intake;

// Two way piston
pros::ADIDigitalOut pistonA('A');
pros::ADIDigitalOut& pistonARef = pistonA;
pros::ADIDigitalOut pistonB('B');
pros::ADIDigitalOut& pistonBRef = pistonB;

// Optical (Port TBD)
pros::Optical optical(15);
pros::Optical& opticalRef = optical;

// Distance
pros::Distance distanceSensor(9);
pros::Distance& distanceRef = distanceSensor;

// Inertial
pros::IMU inertial(8);
pros::IMU& inertialRef = inertial;

// Recording
Recording recording;
Recording& recordingRef = recording;

// Recorder
Recorder recorder(masterRef, recordingRef);


// Player
Player player(recordingRef);

// Dummy Controller for playing autons
DummyController dummy(pros::E_CONTROLLER_MASTER, true, &player);
DummyController& dummyRef = dummy;

// Autonomous
Autonomous autonomousManager(dummyRef, leftMotorsRef, rightMotorsRef, intakeRef, pistonARef, pistonBRef, opticalRef, distanceRef);

// Drive Loop Ticker
Ticker driveTicker(20);

// Decreased Sensitivity Factor (Must be an odd number)
const int decreasedSensitivityFactor = 3;

// Intake Tracker - keeps track of whether intake is in the process of scoring or not.
bool intakeAvailable = true;

// Manual Control
bool manual = false;

// Controller Recording?
bool recordingInput = false;

// Less abrupt intaking
int ticksPerIntakeCheck = 4;
bool intakeCandidateDetected = false;
int ticksSinceIntakeCandidateDetected = 0;

// Drivetrain Velocity
int velocity = 600;

// Button Timer
int manualButtonUnpressableTicks = 0;
int velocityButtonUnpressableTicks = 0;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	colorMode = (colorMode == "blue") ? "red" : "blue";
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	// Pros check if program exists on SD card
	
	pros::lcd::initialize();
	pistonA.set_value(true);
	pistonB.set_value(true);

	// leftMotors.set_brake_modes(MOTOR_BRAKE_BRAKE);
	// rightMotors.set_brake_modes(MOTOR_BRAKE_BRAKE);
	
	pros::lcd::set_text(1, "Initialized!");
	master.print(0, 0, "Manual: %d", manual);
	master.print(0, 1, "Velocity: %d", velocity);

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off._absolute(1000, 600);
 */
void autonomous()
{
	// Incomplete
	// Preferably with actual odom, but if it is accurate enough, we can use this
	// auto drive = ChassisControllerBuilder()
	// 				 .withMotors({11, -12, 13}, {-18, 19, -20})
	// 				 .withDimensions(AbstractMotor::gearset::green, {{4_in * 0.75, 11.5_in, 2.5_in, 1.5_in}, imev5GreenTPR})
	// 				 .withOdometry()
	// 				 .buildOdometry();
	// MotorGroup intake({-1, 10});
	// pros::ADIDigitalOut pistonA('A');
	// pros::ADIDigitalOut pistonB('B');

	// // Test
	// drive->setState({0_ft, 0_ft, 0_deg});
	// drive->driveToPoint({1_ft, 1_ft});
	autonomousManager.run();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{

	while (true)
	{
		driveTicker.startTick();
		master.runUpdate();

		if (recordingInput) {
			recorder.recordUpdate();
		}
		
		// Debug
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
						 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
						 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		// Drive
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		// Prevent drift when idle
		if (abs(left) < 10)
		{
			left = 0;
		}
		if (abs(right) < 10)
		{
			right = 0;
		}

		// Cubic function (as of now), makes small joystick movements less sensitive, for easier maneuverability
		double rawVoltageOutputLeft = pow(left, decreasedSensitivityFactor) / pow(127, (decreasedSensitivityFactor - 1));
		double rawVoltageOutputRight = pow(right, decreasedSensitivityFactor) / pow(127, (decreasedSensitivityFactor - 1));
		/**
		 * The above code is the raw voltage to apply to the motors.
		 * However, I choose to use velocity instead of voltage to control the motors.
		 * 	1 - This is because velocity is more consistent than voltage,
		 * 		which is important when playing the auton
		 * 	2 - Both sides will hopefully spin at the same speed for the same input,
		 * 		something that is not guaranteed with voltage. This will make it
		 * 		easier for the driver.
		 */
		
		// Scale the voltage to the max velocity of the motor
		// double leftVelocity = rawVoltageOutputLeft / 127 * velocity;
		// double rightVelocity = rawVoltageOutputRight / 127 * velocity;

		// turning = abs(right) > 30;
		// if (turning) {
		// 	currentTurnDecrement *= turnDecrement;
		// 	leftVelocity *= currentTurnDecrement;
		// 	rightVelocity *= currentTurnDecrement;
		// 	if (currentTurnDecrement < 0.1) {
		// 		turning = false;
		// 		currentTurnDecrement = 1;
		// 	}
		// } else {
		// 	currentTurnDecrement = 1;
		// }
		// leftMotors.move_velocity(leftVelocity + rightVelocity);
		// rightMotors.move_velocity(leftVelocity - rightVelocity);
		leftMotors.move_velocity(rawVoltageOutputLeft / 127 * velocity);
		rightMotors.move_velocity(rawVoltageOutputRight / 127 * velocity);
		// leftMotors.move(left + right);
		// rightMotors.move(left - right);

		// Intake - manual control
		if (manual) {
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
			if (recordingInput) {
				recorder.saveRecording();
				pros::lcd::set_text(6, "Recording saved!");
			} else {
				recordingInput = true;
			}
		}

		if (master.get_digital(DIGITAL_DOWN))
		{
			recordingInput = true;
			pros::lcd::set_text(6, "Recording started!");
		}

		// Automatic intake scoring - currently works, but delay code not working yet
		if (!manual && distanceSensor.get() < 100) // If distance sensor detects a ring
		{
			// Only start scoring if ring has been detected for a few ticks, to prevent knocking it away
			if (ticksSinceIntakeCandidateDetected > ticksPerIntakeCheck) {

				// This is to prevent the intake from initiating the scoring process multiple times for the same ring
				if (intakeAvailable)
				{
					intake.move_relative(-3692, 600);
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

		if (master.get_digital(DIGITAL_B) && manualButtonUnpressableTicks == 0)
		{
			manual = !manual;
			master.print(0, 0, "Manual: %d", manual);
			manualButtonUnpressableTicks = 10;
		} else {
			if (manualButtonUnpressableTicks > 0) {
				manualButtonUnpressableTicks--;
			}
		}

		if (master.get_digital(DIGITAL_A) && velocityButtonUnpressableTicks == 0)
		{
			velocity = (velocity == 600) ? 200 : 600;
			master.print(1, 1, "Velocity: %d", velocity);
			velocityButtonUnpressableTicks = 10;
		} else {
			if (velocityButtonUnpressableTicks > 0) {
				velocityButtonUnpressableTicks--;
			}
		}

		// Get data for distance sensor for testing purposes
		pros::lcd::set_text(1, to_string(distanceSensor.get()));
		pros::lcd::set_text(2, to_string(intake.get_position()));
		pros::lcd::set_text(3, to_string(inertial.get_heading()));
		driveTicker.waitTick();
	}


}

