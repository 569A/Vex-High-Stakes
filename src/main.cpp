#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <cstddef>

// Drivetrain motor groups
pros::MotorGroup left_tracker({-18}, pros::MotorGearset::blue); // use one of the motors in the left motor group to track position
pros:: MotorGroup left_motor_group({-18, 19, -20});
pros:: MotorGroup right_motor_group({11, -12, 13});

// Intake motors
pros::Motor flex_wheel_intake(9, pros::MotorGearset::blue);
pros::Motor hook_intake(-2, pros::MotorGearset::green);

// Mogo mech piston
pros::adi::DigitalOut mogo_piston('A');

// Doinker
pros::adi::DigitalOut doinker('B');

// Arm motor
pros::Motor arm_motor(10, pros::MotorGearset::red);

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                      
					                 14, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// Input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  14, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);
// Drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              12.625, // 12.625 inch track width
                              lemlib::Omniwheel::NEW_325, 
                              480, // drivetrain rpm is 480
                              8 // horizontal drift is 8 (for now),
);


// Inertial sensor
pros::Imu imu(5);

// // Horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(-3);

// // Vertical tracking wheel encoder
// pros::Rotation vertical_encoder(5);

// // Horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, 2, -4.25);
lemlib::TrackingWheel vertical_tracking_wheel(&left_tracker, lemlib::Omniwheel::NEW_325, -6.3125, 480); //* 1.021276595745269, -6.3125, 480);

// // Vertical tracking wheel
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -2.5);

// odometry settings
// lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
//                             nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
//                             &horizontal_tracking_wheel, // horizontal tracking wheel 1
//                             nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
//                             &imu // inertial sensor
// );

// Odometry sensors
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, //&horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                         &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(5.8, // proportional gain (kP) 4.55 worked well 5.4
                                              0, // integral gain (kI)
                                              82, // derivative gain (kD) 70
                                              0, // anti windup
                                            //   0, // small error range, in inches
                                            //   0, // small error range timeout, in milliseconds
                                            //   0, // large error range, in inches
                                            //     0, // large error range timeout, in milliseconds
                                            //     0 // maximum acceleration (slew)
                                              1, // small error range, in inches
                                              40, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              400, //400 large error range timeout, in milliseconds
                                              0//40 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.825, // proportional gain (kP) 1.9 // 2.8
                                              0, // integral gain (kI)
                                              28, // derivative gain (kD) 30 // 27
                                              0, // anti windup
                                            //   0, // small error range, in degrees
                                            //     0, // small error range timeout, in milliseconds
                                            //     0, // large error range, in degrees
                                            //     0, // large error range timeout, in milliseconds
                                            //     0 // maximum acceleration (slew)
                                              1, // small error range, in degrees
                                              50, // small error range timeout, in milliseconds
                                              4, // large error range, in degrees
                                              205, // 600 large error range timeout, in milliseconds
                                              0 // 40 maximum acceleration (slew)
);

// Create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors						
                        &throttle_curve, 
                        &steer_curve
);

// Optical sensor
pros::Optical optical(15);

// Distance sensor
pros::Distance distance(8);

// Color to score
std::string color = "red";


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);
    arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.calibrate();
    pros::Task screen_task([&]() {
    while (true) {
        // print robot location to the brain screen
        pros::lcd::print(5, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(6, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(7, "Theta: %f", chassis.getPose().theta); // heading
        pros::lcd::print(4, "Arm current: %f", arm_motor.get_current_draw()); 
        // delay to save resources
        pros::delay(50);
    }
});
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

void get_to_point(float x, float y) {
    chassis.turnToPoint(x, y, 10000);
    chassis.moveToPoint(x, y, 10000);
}

double get_intake_closest_to_ready_mogo_score() {
    double current_position = hook_intake.get_position();
    double target_position = 0;
    while (target_position < current_position) {
        target_position += 2957;
    }
    return target_position;
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
 * from where it left off.
 */
void autonomous() {
    // Blue Pos
    chassis.setPose(60, -41, 90);
    chassis.moveToPose(41, -34, 120, 10000, {.forwards = false, .maxSpeed = 70});
    chassis.moveToPose(31, -27, 120, 10000, {.forwards = false, .lead = 0.1, .maxSpeed = 70}, false);
    mogo_piston.set_value(1);
    hook_intake.move(127);
    flex_wheel_intake.move(-127);
    chassis.turnToPoint(24, -48, 10000);
    chassis.moveToPoint(24, -50, 10000);
    chassis.turnToPoint(24, -10, 10000, {}, false);
    chassis.moveToPoint(24, -10, 10000, {}, false);
    left_motor_group.move(50);
    right_motor_group.move(50);
    flex_wheel_intake.move(0);
    hook_intake.move(0);
    return;
    
    // pros::delay(200);
    // // hook_intake.set_zero_position(-1576);
    // // hook_intake.move_absolute(0, 600);
    // // set position to x:0, y:0, heading:0
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(90, 10000);
    // return;
    chassis.setPose(-60.1, 0, 90);
    // turn to face heading 90 with a very long timeout
    // arm_motor.move_absolute(630, 100);
    // while (!(arm_motor.get_position() < arm_motor.get_target_position() + 10 && arm_motor.get_position() > arm_motor.get_target_position() - 10)) {
    //     pros::delay(20);w
    // }

    bool intake_running = false;
    int ticks_since_intake = 0;
    std::string current_ring_color = "none";
    bool should_run_intake = false;
    int wait_ticks = 0;

    bool intake_task_running = false;

    pros::Task auton_intake_manager([&]() {
        while (true) {
            if (optical.get_proximity() > 220) {
                ticks_since_intake = 0;
            }
            ticks_since_intake++;
            if (distance.get() < 200) {
                if (hook_intake.get_current_draw() > 2450 && ticks_since_intake > 12) {
                    hook_intake.move_relative(-400, 200);
                    wait_ticks = 10;
                }
            }
            pros::delay(20);
        }
    });

    arm_motor.set_zero_position(0);
    hook_intake.set_zero_position(0);
    // #1 Score on alliance stake
    arm_motor.move_absolute(140, 100);
    pros::delay(500);
    hook_intake.move_absolute(400, 200);
    pros::delay(600);
    hook_intake.move_relative(-50, 200);
    pros::delay(100);

    // #2 Get mogo
    chassis.moveToPoint(-48, 0, 100000, {}, false);
    chassis.turnToHeading(0, 10000, {}, false);
    // y -24.5
    chassis.moveToPose(-50, -29, 0, 2000, {.forwards = false, .lead = 0.1, .maxSpeed = 70}, false);
    mogo_piston.set_value(1);

    // Activate all intakes
    flex_wheel_intake.move(-127);
    hook_intake.move(127);
    intake_task_running = true;

    pros::delay(200);

    arm_motor.move_absolute(0, 100);
    // Ring 1 by ladder
    chassis.turnToHeading(90, 10000);
    chassis.moveToPoint(-24, -24, 10000);

    // Get to this point instead of directly to ring #2 to avoid hitting the ladder
    get_to_point(0, -45);

    // Ring 2 by high stake
    chassis.turnToPoint(26, -46, 10000);
    chassis.moveToPoint(26, -46, 10000);

    // Ring 3 - Maybe use this for high stake?
    chassis.turnToPoint(0, -55, 10000);
    chassis.moveToPoint(0, -55, 10000);
    
    // Ring 4
    chassis.turnToPoint(-24, -46, 10000);
    chassis.moveToPoint(-24, -46, 10000);

    // Ring 5-6 (cluster of 3 rings, this is the horizontal 2)
    // can just combine into one motion because they are on the same path
    // chassis.turnToPoint(-48, -48, 10000);
    // chassis.moveToPoint(-48, -48, 10000);
    // chassis.turnToPoint(-60, -48, 10000);
    // chassis.moveToPoint(-60, -48, 10000);

    // chassis.turnToPoint(-55, -48, 10000);
    // chassis.moveToPoint(-55, -48, 10000);
    chassis.turnToPoint(-57, -46, 10000);
    chassis.moveToPoint(-57, -46, 10000);
    // Move back for ring 7
    chassis.turnToHeading(270, 10000);
    chassis.moveToPoint(-50, -46, 10000, {.forwards = false});
    
    // Ring 7
    chassis.turnToPoint(-48, -61, 10000);
    chassis.moveToPoint(-48, -61, 10000);

    // uncomment from here for ring 7
    chassis.turnToHeading(68.2, 10000);
    // chassis.turnToPoint(-48, -53, 10000);
    chassis.moveToPoint(-37, -54, 10000);
    
    //chassis.moveToPoint(-38, -48, 10000, {.forwards = false});
    chassis.turnToHeading(180, 10000);
    chassis.moveToPoint(-42, -46, 10000, {.forwards = false});
    
    chassis.turnToPoint(100, 100, 10000, {}, false);

    left_motor_group.move(-100);
    right_motor_group.move(-100);
    // #3 Drop mogo in corner
    pros::delay(400);
    left_motor_group.move(0);
    right_motor_group.move(0);
    chassis.setPose(-51, -51, chassis.getPose().theta);
    mogo_piston.set_value(0);

    // chassis.turnToPoint(-42, -42, 10000);
    // chassis.moveToPoint(-42, -42, 10000);
    // chassis.turnToPoint(-48, -48, 10000);
    // chassis.moveToPoint(-48, -48, 10000);
    chassis.turnToHeading(45, 1000, {}, false);
    chassis.moveToPoint(-38, -42, 10000, {}, false);
    chassis.turnToHeading(0 , 10000, {}, false);
    mogo_piston.set_value(1);

    // Wall reset pose
    left_motor_group.move(-100);
    right_motor_group.move(-100);
    pros::delay(1000);
    chassis.setPose(-39, -64, chassis.getPose().theta);
    left_motor_group.move(0);
    right_motor_group.move(0);
    
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y + 10, 10000, {}, false);
    mogo_piston.set_value(0);

    chassis.turnToPoint(-48, 0, 10000);
    chassis.moveToPoint(-48, 0, 10000);

    chassis.turnToHeading(180, 10000);

    // // get_to_point(-48, 0);
    // // chassis.turnToPoint(-48, -1000, 10000);

    // // #4 Get mogo 2
    chassis.moveToPose(-48, 27.5, 180, 10000, {.forwards = false, .lead = 0.1, .maxSpeed = 60});
    mogo_piston.set_value(1);
    
    // Activate all intakes 2nd run
    flex_wheel_intake.move(-127);
    hook_intake.move(80);
    intake_task_running = true;

    pros::delay(200);

    arm_motor.move_absolute(0, 100);
    // Ring 1 by ladder
    chassis.turnToHeading(90, 10000);
    chassis.moveToPoint(-24, 24, 10000);

    // Get to this point instead of directly to ring #2 to avoid hitting the ladder
    get_to_point(0, -45);

    // Ring 2 by high stake
    chassis.turnToPoint(26, 48, 10000);
    chassis.moveToPoint(26, 48, 10000);

    // Ring 3 - Maybe use this for high stake?
    chassis.turnToPoint(0, 55, 10000);
    chassis.moveToPoint(0, 55, 10000);
    
    // Ring 4
    chassis.turnToPoint(-24, 46, 10000);
    chassis.moveToPoint(-24, 46, 10000);

    // Ring 5-6 (cluster of 3 rings, this is the horizontal 2)
    // can just combine into one motion because they are on the same path
    // chassis.turnToPoint(-48, -48, 10000);
    // chassis.moveToPoint(-48, -48, 10000);
    // chassis.turnToPoint(-60, -48, 10000);
    // chassis.moveToPoint(-60, -48, 10000);

    // chassis.turnToPoint(-55, -48, 10000);
    // chassis.moveToPoint(-55, -48, 10000);
    chassis.turnToPoint(-57, 46, 10000);
    chassis.moveToPoint(-57, 46, 10000);
    // Move back for ring 7
    chassis.turnToHeading(-90, 10000);
    chassis.moveToPoint(-50, 46, 10000, {.forwards = false});
    
    // Ring 7
    chassis.turnToPoint(-48, 61, 10000);
    chassis.moveToPoint(-48, 61, 10000);

    // uncomment from here for ring 7
    chassis.turnToHeading(31.8, 10000);
    // chassis.turnToPoint(-48, -53, 10000);
    chassis.moveToPoint(-37, 54, 10000);
    
    //chassis.moveToPoint(-38, -48, 10000, {.forwards = false});
    chassis.turnToHeading(0, 10000);
    chassis.moveToPoint(-42, 48, 10000, {.forwards = false});
    
    chassis.turnToPoint(-100, -100, 10000, {}, false);

    left_motor_group.move(-100);
    right_motor_group.move(-100);
    // #3 Drop mogo in corner
    pros::delay(400);
    left_motor_group.move(0);
    right_motor_group.move(0);
    chassis.setPose(-51, 51, chassis.getPose().theta);
    mogo_piston.set_value(0);

    // chassis.turnToPoint(-42, -42, 10000);
    // chassis.moveToPoint(-42, -42, 10000);
    // chassis.turnToPoint(-48, -48, 10000);
    // chassis.moveToPoint(-48, -48, 10000);
    chassis.turnToHeading(45, 1000, {}, false);
    chassis.moveToPoint(-34.5, -42, 10000, {}, false);
    chassis.turnToHeading(0 , 10000, {}, false);
    mogo_piston.set_value(1);
    left_motor_group.move(-100);
    right_motor_group.move(-100);
    pros::delay(1000);
    chassis.setPose(-36, -64, chassis.getPose().theta);
    left_motor_group.move(0);
    right_motor_group.move(0);
    
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y + 10, 10000, {}, false);
    mogo_piston.set_value(0);
    // mogo_piston.set_value(1);

    // get_to_point(-24, 24);
    // get_to_point(24, 48);
    // get_to_point(0, 60);
    // get_to_point(-24, 48);
    // get_to_point(-48, 48);
    // get_to_point(-60, 48);


    // Angular PID test 
    // chassis.turnToHeading(45, 100000, {}, false);
    // chassis.turnToHeading(90, 100000, {}, false);
    // chassis.turnToHeading(135, 100000, {}, false);
    // chassis.turnToHeading(180, 100000, {}, false);
    // chassis.turnToHeading(225, 100000, {}, false);
    // chassis.turnToHeading(270, 100000, {}, false);
    // chassis.turnToHeading(315, 100000, {}, false);
    // chassis.turnToHeading(360, 100000, {}, false);
    // chassis.turnToHeading(315, 100000, {}, false);
    // chassis.turnToHeading(270, 100000, {}, false);
    // chassis.turnToHeading(225, 100000, {}, false);
    // chassis.turnToHeading(180, 100000, {}, false);
    // chassis.turnToHeading(135, 100000, {}, false);
    // chassis.turnToHeading(90, 100000, {}, false);
    // chassis.turnToHeading(45, 100000, {}, false);
    // chassis.turnToHeading(0, 100000, {}, false);


    // chassis.moveToPoint(10, 48, 10000, {}, false);
    // chassis.moveToPoint(0, 0, 10000, {.forwards = false}, false);
    // chassis.turnToPoint(0, 0, 10000);
    // chassis.moveToPose(0, 0, 0, 10000);
    // // chassis.moveToPoint(10, 48, 10000);
    // chassis.moveToPoint(0, 0, 10000);
    // chassis.moveToPoint(20, 48, 10000);

    // chassis.moveToPose(0, 48, 0, 10000);
    // chassis.moveToPose(10, 48, 0, 10000);
    // chassis.moveToPose(0, 0, 0, 10000);

    // chassis.moveToPose(0, 48, 0, 10000);
    // chassis.moveToPose(10, 48, 0, 10000);
    // chassis.moveToPose(0, 0, 0, 10000);

    //     chassis.moveToPose(0, 48, 0, 10000);
    // chassis.moveToPose(10, 48, 0, 10000);
    // chassis.moveToPose(0, 0, 0, 10000);

    //     chassis.moveToPose(0, 48, 0, 10000);
    // chassis.moveToPose(10, 48, 0, 10000);
    // chassis.moveToPose(0, 0, 0, 10000);
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
void opcontrol() {

	optical.disable_gesture();

    bool digital_x_was_pressed = false;
    bool digital_up_was_pressed = false;
    bool digital_down_was_pressed = false;
    bool digital_b_was_pressed = false;
    bool digital_y_was_pressed = false;

    bool flex_wheel_running = false;
    bool intake_running = false;
    int last_target = 0;
    int ticks_since_intake = 0;
    int wait_ticks = 10;
    std::string current_ring_color = "none";
    bool return_flag = false;

    bool should_run_intake = false;

    int arm_state = 0;
    // 0 = resting, 1 = ready to intake for wall stakes, 2 = ready to score on wall stakes

    // High stake scoring
    bool ready_to_score = false;
    bool scoring = false;
    bool get_arm_ready_to_score = false;
    int arm_wait_ticks = 10;

    // Keep track of mogo mech
    bool mogo_clamped = false;

    // Keep track of the doinker
    bool doinker_down = false;

    bool next_ring_must_reverse = false;
    chassis.setPose(-60.1, 0, 90);
    // turn to face heading 90 with a very long timeout
    // arm_motor.move_absolute(630, 100);
    // while (!(arm_motor.get_position() < arm_motor.get_target_position() + 10 && arm_motor.get_position() > arm_motor.get_target_position() - 10)) {
    //     pros::delay(20);w
    // }
    arm_motor.move_absolute(140, 100);
    pros::delay(500);
    hook_intake.move_absolute(1300, 200);
    pros::delay(600);
    arm_motor.move_absolute(-10, 200);

    flex_wheel_intake.move_velocity(-500);

	while (true) {
		optical.set_led_pwm(100);
		// Tank drive
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        chassis.tank(leftY, rightY);

		// Intake control
		pros::lcd::set_text(0, "Optical: " + std::to_string(optical.get_hue()));
        pros::lcd::set_text(1, "Intake: " + std::to_string(hook_intake.get_position()));
        pros::lcd::set_text(2, "Arm: " + std::to_string(arm_motor.get_position()));
        pros::lcd::set_text(3, "Intake Current: " + std::to_string(hook_intake.get_current_draw()));

        // Flex wheel intake
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            flex_wheel_intake.move_velocity(-500);
            flex_wheel_running = true;
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            flex_wheel_intake.move_velocity(500);
            flex_wheel_running = false;
        }    

        /**
        The intake control logic is different based on what we are trying to do. For example, here, in state 0 (resting position) 
        we try to score rings on mogos and color sort appropriately.
        In state 1 (intaking rings to score on wall stakes), the intake is stopped and activates to pull the ring in once the driver pushes a button.
        In state 2 (ready to score on wall stakes), the intake is stopped and the rings should be ready to flip onto the stake.
        At this point, the driver can push a button to score the rings.
         */
        if (arm_state == 0) {
            ready_to_score = false;

            /**
            Here we use the distance sensor to check if the mogo has 5 rings, which can then determine if the next ring is the 6th ring.
            The hook gets caught on it after scoring it, which isn't good, so we reverse the intake after scoring it to prevent this from happening.
            Setting this value to true will reverse the intake on scoring the 6th ring. (later in the code)
            */
            if (distance.get() < 200) {
                next_ring_must_reverse = true;
            } else {
                next_ring_must_reverse = false;
            }
            // Hook intake
            if (optical.get_proximity() > 220 ) {
                // TODO, if intake is already scoring a ring that we want, but then a ring comes in that is the wrong
                // color, we should throw the good ring out.
                if (intake_running) {

                }
                intake_running = true;

                // Check color of ring - make sure to adjust on tournament day because this changes with light conditions.
                if (optical.get_hue() > 0 && optical.get_hue() < 15 ) {
                    current_ring_color = "red";
                }
                
                if (optical.get_hue() > 200 && optical.get_hue() < 235 ) {
                    current_ring_color = "blue";
                }
                ticks_since_intake = 0;

            } else {
                if (flex_wheel_running) {
                    flex_wheel_intake.move_velocity(-500);
                }
            }

            // We use this to track the amount of time since we intaked in order to know when to color sort.
            ticks_since_intake++;

            /**
            This is probably the most unique part of our code. To color sort, we have to either add a mechanism that
            makes the ring not score on the mogo or do it in code so that the ring flies off on its own. A mechanism would
            be complex and would just add more weight to the robot, so we decided to do it in code. This is based on physics
            and how objects in motion tend to stay in motion. If the ring is being moved upwards, but at the moment it is flipped
            the intake stops, the ring will continue to move upwards and fly off instead of going on the mogo.

            The first issue with implementation was that stopping the color sorting just a couple degrees of the motor too late or too early would not be
            sudden enough to make the ring fly off.

            The first attempt included stopping the intake once it reached a certain range of about where the intake should stop
            to throw the ring off, but this introduced multiple problems. If we ran multiple hooks on our intake, we would have no idea
            which one the ring would be on and thus we would have no idea when to shoot the ring off. Additionally, if the robot started
            with the hook just a little bit off from where it should have started, the intake would always stop too late or too early,
            causing the same problem of scoring the ring on the mogo instead of throwing it off.

            The second attempt was time based, and after the optical sensor lost sight of the ring, we would stop the intake in x amount of time.
            But, this ran into the same problem as the first attempt, where the intake would stop too late or too early, and if the intake jammed momentarily,
            it would be even less accurate.

            This final iteration uses the current draw of the intake motor to determine when to stop the intake. Rings are very light, so it does not take much electric
            current to power the motor that moves it along on a conveyor belt. But at the moment the ring gets flipped 180 degrees and slapped onto the stake of the mogo, much more
            power is required. After experimenting, we found that the intake motor draws around 2100 mA when the ring is in the moment of being flipped onto the stake, but only a few hundred mA when just moving the ring.
            This is very consistent over time. However, the intake motor also draws a lot of power when lifting the ring from the flex wheel intake to the hook intake. We do not want to mistake this as a time to
            stop the intake for color sorting. This is done by running the color sorter stop only when 0.3 seconds have passed since the optical sensor
            lost sight of the ring.

             */
            if (current_ring_color != color && current_ring_color != "none") {
                if (hook_intake.get_current_draw() > 2050 && ticks_since_intake > 15) {
                    hook_intake.move(0);
                    wait_ticks = 10; // The next step is waiting a bit before the intake runs again. If we don't wait a bit, the intake
                                     // will stop for so little time it will not throw the ring off.
                }            
            }
            // wait_ticks is used to wait a bit before the intake runs again. Every 20 milliseconds, we decrement it by 1, and when it reaches 0, the intake runs again.
            if (wait_ticks > 0) {
                wait_ticks--;
            }
            if (wait_ticks == 0) {
                // Repower the intake
                if (should_run_intake) {
                    hook_intake.move(126);
                }
            }

            // Manual control in case of emergency to reverse/turn back on the intake (ring gets stuck, etc.)
            // The buttons have to be held down to work instead of toggling (for simplicity)
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                should_run_intake = true;
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                hook_intake.move(-126);
                should_run_intake = false; // Override running the intake forward
            }
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
                hook_intake.move(0); 
                should_run_intake = false; // Override running the intake forward
            } else if (next_ring_must_reverse) {
                if (hook_intake.get_current_draw() > 2350 && ticks_since_intake > 30) {
                    hook_intake.move_relative(-400, 200);
                    should_run_intake = false;
                }
            } else {
                should_run_intake = true;
            }
        } else if (arm_state == 1 && arm_motor.get_position() + 20 > arm_motor.get_target_position() && arm_motor.get_position() - 20 < arm_motor.get_target_position()) { // This contains a position check of the arm motor to make sure it is at the target position before the hook intake is moved to the right spot
            if (!ready_to_score) {      
                ready_to_score = true;
                // We just need to move the intake so that one of the hooks is in the right spot to reverse intake a ring for wall stake scoring.
                // This function (see above) allows us to do this.
                hook_intake.move_absolute(get_intake_closest_to_ready_mogo_score() - 3385, 600); 
                ticks_since_intake = 0;
                get_arm_ready_to_score = true;
                last_target = last_target + 2955;
            }
        } else if (arm_state == 2) {
            flex_wheel_intake.move_velocity(600);
            ready_to_score = false;
            /**
            High stake scoring - we don't want to have to manually reverse the intake for scoring on wall stakes
            and change it back for mogos, so this is done with a macro. At the press of a button, the intake
            will score loaded rings onto the wall stakes.
            */
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && !digital_b_was_pressed) {
                digital_b_was_pressed = true;
                hook_intake.move_relative(-2500, 200);
                last_target = last_target - 2955;
            } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                digital_b_was_pressed = false;
            }
        }

        /**
         Toggleable Color Sorting - If we forget to set the color sort color before the match, we
         don't want to go the entire match unable to score a single ring because they all get tossed.
         This allows us to change the color of the rings we want to score at the press of a button.

         If we absolutely have to run a negative corner play to win, this will also be needed.
        */
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && !digital_x_was_pressed) {
            digital_x_was_pressed = true;
            if (color == "red") {
                color = "blue";
            } else if (color == "blue") {
                color = "none";
            } else {
                color = "red";
            }
            master.clear_line(0);
            master.set_text(0, 0, "Ring color: " + color);        
        } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            digital_x_was_pressed = false;
        }

        /**
        Mogo mech - This is pretty simple, just turn it on at the press of one button, off at the other. 
        Maybe due to physical constraints, we will need to start with the mogo closed, so keep in mind for future.
        We also keep track of whether the mogo is clamped or not, to maybe in the future not score rings, but ready them in the intake
        if we don't even have a mogo.
        */
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            mogo_piston.set_value(0);
            mogo_clamped = false;
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            mogo_piston.set_value(1);
            mogo_clamped = true;
        }

        /**
        Doinker - This code simply activates/deactivates the doinker at the press of the y button.
         */
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && !digital_y_was_pressed) {
            digital_y_was_pressed = true;
            doinker.set_value(!doinker_down);
            doinker_down = !doinker_down;
        } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            digital_y_was_pressed = false;
        }

        /**
        Arm control - The highest arm position is not set right yet, it undershoots what it should be, but 
        it functions as it should. Pressing the up arrow on the controller moves the arm up a bit so that it
        can collect rings on the back of the intake. Pressing the up arrow again moves it to the very top position
        where the arm moves the intake to the same height as the wall stakes and can score rings on them. (see above macro code)
         */
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && !digital_up_was_pressed) {
            get_arm_ready_to_score = false;
            if (arm_state == 0) {
                arm_motor.move_absolute(775, 100);
                arm_state++;
            } else if (arm_state == 1) {
                hook_intake.move_relative(-700, 200);
                arm_motor.move_absolute(2570, 35);
                arm_state++;
            } 
            digital_up_was_pressed = true;
        } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            digital_up_was_pressed = false;
        }
        // Down arrow does the opposite.
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && !digital_down_was_pressed) {
           get_arm_ready_to_score = false;
           flex_wheel_intake.move(-500);
            if (arm_state == 1) {
                arm_motor.move_absolute(0, 80);
                arm_state--;
            } else if (arm_state == 2) {
                arm_motor.move_absolute(775, 100);
                arm_state--;
            }
            digital_down_was_pressed = true;
        } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            digital_down_was_pressed = false;
        }


        pros::delay(20);
	}
}