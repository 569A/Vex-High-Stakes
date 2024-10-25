#include "main.h"
#include "DummyController.h"
#include "Autonomous.h"
#include "Recording.h"
#include "Ticker.h"
#include "SlewRateController.h"
#include <cmath>

using namespace std;

string colorMode = "blue";

	
// Controller
ControllerBase master(pros::E_CONTROLLER_MASTER);
ControllerBase& masterRef = master;


// Motor groups for drive
okapi::MotorGroup leftMotors({-11, 12, -13});
okapi::MotorGroup& leftMotorsRef = leftMotors;

okapi::MotorGroup rightMotors({18, -19, 20});
okapi::MotorGroup& rightMotorsRef = rightMotors;

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

// Position tracking (EXPERIMENTAL)
double x = 0;
double y = 0;
double z = 0;

double lastXVelocity = 0;
double lastYVelocity = 0;
double lastZVelocity = 0;

double theta = 0;

// Slew Rate Controller for Drive
SlewRateController leftDriveSlewRateController(60);
SlewRateController rightDriveSlewRateController(60);

// Stage for recording
int stage = 1;

// Competition Mode
bool competitionMode = false;

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
	pros::lcd::initialize();

	// Mogo Mechanism open by default
	pistonA.set_value(true);
	pistonB.set_value(true);
	
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
void competition_initialize() {
	competitionMode = true;
}
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
	// We will be using my rerun implementation this tournament.
	/**
	 * Stage System:
	 * 	Driver runs are stored in files on the sd card. Each file is a stage.
	 * 	Each stage is a different part of the auton.
	 *  Running autonomousManager.run() plays the next stage. Stages are determined by their
	 *  file name on the sd card, and are read and saved as such.
	 */
	autonomousManager.run(); // Runs the first stage of the auton
	intake.move_relative(-1896, 600);
	autonomousManager.run(); // Runs the next stage of the auton

	// Incomplete
	// Preferably with actual odom, but if it is accurate enough, we can use this
	// auto drive = ChassisControllerBuilder()
	// 				 .withMotors(leftMotors, rightMotors)
	// 				 .withDimensions(AbstractMotor::gearset::blue, {{3.25_in, 12.5625_in}, 300.0 * (48.0/36.0)})
	// 				 .withOdometry()
	// 				 .buildOdometry();

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
	bool digitalUpButtonHeld = false;
	bool digitalDownButtonHeld = false;

	bool digitalLeftButtonHeld = false;
	bool digitalRightButtonHeld = false;

	bool digitalBButtonHeld = false;
	bool digitalAButtonHeld = false;

	while (true)
	{
		driveTicker.startTick();
		master.runUpdate();

		if (recordingInput) {
			recorder.recordUpdate();
		}
		
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
		
		int leftVelocity = rawVoltageOutputLeft / 127 * velocity;
		int rightVelocity = rawVoltageOutputRight / 127 * velocity;
		if (recordingInput) {	
			leftMotors.moveVelocity(leftDriveSlewRateController.calculate(leftVelocity));
			rightMotors.moveVelocity(rightDriveSlewRateController.calculate(rightVelocity));
		} else {
			leftMotors.moveVelocity(leftVelocity);
			rightMotors.moveVelocity(rightVelocity);
		}

		// Intake - manual control
		if (manual) {
			intakeAvailable = true;
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

		// Saves recording
		if (master.get_digital(DIGITAL_UP) && !competitionMode)
		{
			if (!digitalUpButtonHeld) {
				digitalUpButtonHeld = true;
				if (recordingInput) {
					recorder.saveRecording(stage);
					pros::lcd::set_text(6, "Recording saved!");
				} else {
					recordingInput = true;
				}
			} 
		} else {
			digitalUpButtonHeld = false;
		}

		// Starts recording
		if (master.get_digital(DIGITAL_DOWN) && !competitionMode)
		{
			if (!digitalDownButtonHeld) {
				digitalDownButtonHeld = true;
				recordingInput = true;
				pros::lcd::set_text(6, "Recording started!");
			}
		} else {
			digitalDownButtonHeld = false;
		}

		// Selects which stage of auton to record (begins at 0, can record as many as you want)
		if (master.get_digital(DIGITAL_LEFT) && !competitionMode)
		{
			if (!digitalLeftButtonHeld) {
				digitalLeftButtonHeld = true;
				// Stage cannot be less than 0
				stage = (stage - 1 < 0) ? 0 : stage - 1;
				master.print(2, 0, "Stage: %d", stage);
			}
			
		} else {
			digitalLeftButtonHeld = false;

		}
		if (master.get_digital(DIGITAL_RIGHT) && !competitionMode)
		{
			if (!digitalRightButtonHeld) {
				digitalRightButtonHeld = true;
				
				stage = (stage + 1 > 5) ? 5 : stage + 1;
				master.print(1, 0, "Stage: %d", stage);
			}
		} else {
			digitalRightButtonHeld = false;
		}

		// Automatic intake scoring - currently works, but delay code not working yet
		if (!manual && distanceSensor.get() < 100) // If distance sensor detects a ring
		{
			// Only start scoring if ring has been detected for a few ticks, to prevent knocking it away
			if (ticksSinceIntakeCandidateDetected > ticksPerIntakeCheck) {

				// This is to prevent the intake from initiating the scoring process multiple times for the same ring
				if (intakeAvailable)
				{
					intake.move_relative(2250, 600);
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

		if (master.get_digital(DIGITAL_B))
		{
			if (!digitalBButtonHeld) {
				digitalBButtonHeld = true;
				manual = !manual;
				master.print(0, 0, "Manual: %d", manual);
			}
		} else {
			digitalBButtonHeld = false;
		}

		if (master.get_digital(DIGITAL_A) && !competitionMode)
		{
			if (!digitalAButtonHeld) {
				digitalAButtonHeld = true;
				velocity = (velocity == 600) ? 200 : 600;
				master.print(1, 1, "Velocity: %d", velocity);
			}		
		} else {
			digitalAButtonHeld = false;
		}

		// Get data for distance sensor for testing purposes
		pros::lcd::set_text(1, to_string(distanceSensor.get()));
		pros::lcd::set_text(2, to_string(intake.get_position()));
		pros::lcd::set_text(3, to_string(inertial.get_heading()));
		driveTicker.waitTick();
	}


}

