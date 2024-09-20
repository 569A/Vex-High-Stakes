#include "main.h"
#include "DummyController.h"
#include "Autonomous.h"
#include "Recording.h"

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
pros::MotorGroup intake({-1, 10});
pros::MotorGroup& intakeRef = intake;

// Two way piston
pros::ADIDigitalOut pistonA('A');
pros::ADIDigitalOut& pistonARef = pistonA;
pros::ADIDigitalOut pistonB('B');
pros::ADIDigitalOut& pistonBRef = pistonB;

// Optical (Port TBD)
pros::Optical optical(15);
pros::Optical& opticalRef = optical;

// Recording
Recording recording;
Recording& recordingRef = recording;

// Recorder
Recorder recorder(masterRef, recordingRef);


// Player
Player player(recordingRef);

// Dummy Controller for playing autons
DummyController dummy(pros::E_CONTROLLER_MASTER, true, &player);

// Autonomous
Autonomous autonomousManager(dummy, leftMotorsRef, rightMotorsRef, intakeRef, pistonARef, pistonBRef, opticalRef);

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
	
	pros::lcd::set_text(1, "Initialized!");

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
 * from where it left off.
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
		master.runUpdate();
		recorder.recordUpdate();
		
		// Debug
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
						 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
						 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		// Drive
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);
		leftMotors = left;
		rightMotors = right;

		// Intake
		if (master.get_digital(DIGITAL_L1))
		{
			// Velocity depends on gearset 
			intake.move_velocity(200);
		}
		else if (master.get_digital(DIGITAL_L2))
		{
			intake.move_velocity(-200);
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
			recorder.saveRecording();
			pros::lcd::set_text(1, "Recording saved!");
		}

		pros::delay(20); // Never change this!
	}

	// Optical
	pros::lcd::set_text(1, to_string(optical.get_hue()));
}
