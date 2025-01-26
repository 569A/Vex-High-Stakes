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
                              8 // horizontal drift is 8 (for now)
);


// Inertial sensor
pros::Imu imu(5);

// // Horizontal tracking wheel encoder
// pros::Rotation horizontal_encoder(4);

// // Vertical tracking wheel encoder
// pros::Rotation vertical_encoder(5);

// // Horizontal tracking wheel
// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&left_tracker, lemlib::Omniwheel::NEW_325 * 1.021276595745269, -6.3125, 480);

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
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(5.85, // proportional gain (kP) 4.55 worked well 5.4
                                              0, // integral gain (kI)
                                              82, // derivative gain (kD) 70
                                              0, // anti windup
                                            //   0, // small error range, in inches
                                            //   0, // small error range timeout, in milliseconds
                                            //   0, // large error range, in inches
                                            //     0, // large error range timeout, in milliseconds
                                            //     0 // maximum acceleration (slew)
                                              1, // small error range, in inches
                                              5, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              10, //400 large error range timeout, in milliseconds
                                              0//40 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.8, // proportional gain (kP) 1.9 // 2.8
                                              0, // integral gain (kI)
                                              28, // derivative gain (kD) 30 // 27
                                              0, // anti windup
                                            //   0, // small error range, in degrees
                                            //     0, // small error range timeout, in milliseconds
                                            //     0, // large error range, in degrees
                                            //     0, // large error range timeout, in milliseconds
                                            //     0 // maximum acceleration (slew)
                                              3, // small error range, in degrees
                                              10, // small error range timeout, in milliseconds
                                              10, // large error range, in degrees
                                              50, // 600 large error range timeout, in milliseconds
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
        target_position += 2955;
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
    // hook_intake.set_zero_position(-1576);
    //hook_intake.move_absolute(0, 600);
    // set position to x:0, y:0, heading:0
    chassis.setPose(-60.5, 0, 90);
    // turn to face heading 90 with a very long timeout
    // arm_motor.move_absolute(630, 100);
    // while (!(arm_motor.get_position() < arm_motor.get_target_position() + 10 && arm_motor.get_position() > arm_motor.get_target_position() - 10)) {
    //     pros::delay(20);w
    // }
    arm_motor.move_absolute(140, 100);
    pros::delay(500);
    hook_intake.move_absolute(1200, 200);
    pros::delay(600);
    chassis.moveToPoint(-47, 0, 100000);
    chassis.turnToHeading(0, 10000);
    chassis.moveToPoint(-48, -24.5, 10000, {.forwards = false, .maxSpeed = 60}, false);
    mogo_piston.set_value(1);

    flex_wheel_intake.move(-127);
    hook_intake.move(127);

    pros::delay(200);

    arm_motor.move_absolute(0, 100);

    chassis.turnToHeading(90, 10000);
    chassis.moveToPoint(-24, -24, 10000);

    get_to_point(0, -45);

    chassis.turnToPoint(24, -48, 10000);
    chassis.moveToPoint(24, -48, 10000);

    chassis.turnToPoint(0, -55, 10000);
    chassis.moveToPoint(0, -55, 10000);
    
    chassis.turnToPoint(-24, -48, 10000);
    chassis.moveToPoint(-24, -48, 10000);

    chassis.turnToPoint(-48, -48, 10000);
    chassis.moveToPoint(-48, -48, 10000);
    chassis.moveToPoint(-60, -48, 10000);
    chassis.moveToPoint(-36, -48, 10000, {.forwards = false});
    chassis.turnToPoint(-48, -60, 10000);
    chassis.moveToPoint(-48, -60, 10000);
    chassis.moveToPoint(-38, -48, 10000, {.forwards = false});
    chassis.turnToPoint(72, 72, 10000);
    if (hook_intake.get_voltage() > 2000) {
        hook_intake.move_relative(-400, 200);
    }
    chassis.moveToPoint(-52.5, -52.5, 10000, {.forwards = false}, false);

    mogo_piston.set_value(0);

    chassis.turnToPoint(-42, -42, 10000);
    chassis.moveToPoint(-42, -42, 10000);
    get_to_point(-48, 0);
    chassis.turnToPoint(-48, -1000, 10000);

    chassis.moveToPoint(-48, 14, 10000, {.forwards = false, .maxSpeed = 30});
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

    bool flex_wheel_running = false;
    bool intake_running = false;
    int last_target = 0;
    int ticks_since_intake = 0;
    int wait_ticks = 10;
    std::string current_ring_color = "none";
    bool return_flag = false;

    int arm_state = 0;
    // 0 = resting, 1 = ready to intake for wall stakes, 2 = ready to score on wall stakes

    // High stake scoring
    bool ready_to_score = false;
    bool scoring = false;
    bool get_arm_ready_to_score = false;
    int arm_wait_ticks = 10;

    // Keep track of mogo mech
    bool mogo_clamped = false;

    bool next_ring_must_reverse = false;
    chassis.setPose(-60.5, 0, 90);
    // turn to face heading 90 with a very long timeout
    // arm_motor.move_absolute(630, 100);
    // while (!(arm_motor.get_position() < arm_motor.get_target_position() + 10 && arm_motor.get_position() > arm_motor.get_target_position() - 10)) {
    //     pros::delay(20);w
    // }
    arm_motor.move_absolute(140, 100);
    pros::delay(500);
    hook_intake.move_absolute(1576, 200);
    pros::delay(600);
    hook_intake.move_absolute(0, 200);

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
            flex_wheel_intake.move(0);
            flex_wheel_running = false;
        }    

        if (arm_state == 0) {
            ready_to_score = false;
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                hook_intake.move(126);
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                hook_intake.move(-126);
            }

            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
                hook_intake.move(0);
            }
            if (distance.get() < 200) {
                next_ring_must_reverse = true;
            } else {
                next_ring_must_reverse = false;
            }
            // Hook intake
            if (optical.get_proximity() > 230 ) { //&& !intake_running && !ready_to_score) {
                if (intake_running) {

                }
                intake_running = true;
                // Check color of ring
                if (optical.get_hue() > 0 && optical.get_hue() < 15 ) {
                    current_ring_color = "red";
                }
                
                if (optical.get_hue() > 200 && optical.get_hue() < 235 ) {
                    current_ring_color = "blue";
                }
                ticks_since_intake = 0;
                    // End

                // if (arm_state == 0 && mogo_clamped) {
                //     hook_intake.move_absolute(last_target + 2955, 600);
                //     ticks_since_intake = 0;

                //     last_target = last_target + 2955;
                //     intake_running = true;
                //} else
                // if (arm_state == 1) {
                //     hook_intake.move_absolute(last_target - 400, 600);
                //     ticks_since_intake = 0;
                //     get_arm_ready_to_score = true;
                //     last_target = last_target + 2955;
                //     ready_to_score = true;
                
                // }

            } else {
                if (flex_wheel_running) {
                    flex_wheel_intake.move_velocity(-500);
                }
            }

            ticks_since_intake++;
            if (current_ring_color != color) {
                if (hook_intake.get_current_draw() > 2050 && ticks_since_intake > 15) {
                    hook_intake.move(0);
                    wait_ticks = 10;
                }            
            }
            if (wait_ticks > 0) {
                wait_ticks--;
            }
            if (wait_ticks == 0) {
                hook_intake.move(126);
            }

            if (next_ring_must_reverse) {
                if (hook_intake.get_current_draw() > 2350 && ticks_since_intake > 30) {
                    hook_intake.move_relative(-400, 200);
                }
            }
        } else if (arm_state == 1 && arm_motor.get_position() + 20 > arm_motor.get_target_position() && arm_motor.get_position() - 20 < arm_motor.get_target_position()) {
            if (!ready_to_score) {

            
            ready_to_score = true;
            hook_intake.move_absolute(get_intake_closest_to_ready_mogo_score() - 3355, 600);  //last_target - 400, 600);
            ticks_since_intake = 0;
            get_arm_ready_to_score = true;
            last_target = last_target + 2955;
            }
        } else if (arm_state == 2) {
            ready_to_score = false;
            // High stake scoring
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && !digital_b_was_pressed) {
                digital_b_was_pressed = true;
                hook_intake.move_relative(-2500, 200);
                last_target = last_target - 2955;
            } else {
                digital_b_was_pressed = false;
            }
        }

        // Toggleable Color Sorting
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && !digital_x_was_pressed) {
            digital_x_was_pressed = true;
            color == "red" ? color = "blue" : color = "red";   
            master.set_text(0, 0, "Ring color: " + color);        
        } else {
            digital_x_was_pressed = false;
        }

        // if (intake_running) {
        //     ticks_since_intake++;
        //     if (current_ring_color != color) {
        //         if (hook_intake.get_current_draw() > 2000 && ticks_since_intake > 30) {
        //             hook_intake.move(0);
        //             return_flag = true;
        //         }            
        //         if (return_flag) {
        //             if (wait_ticks > 0) {
        //                 wait_ticks--;
        //             } else {
        //                 hook_intake.move_absolute(last_target, 600);
        //                 intake_running = false;
        //                 return_flag = false;
        //                 wait_ticks = 10;
        //             }
        //         }
        //     } else {
        //         if (hook_intake.get_position() < hook_intake.get_target_position() + 50 && hook_intake.get_position() > hook_intake.get_target_position() - 50) {
        //             intake_running = false;
        //         }
        //     }
        // }

        // Mogo mech
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            mogo_piston.set_value(0);
            mogo_clamped = false;
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            mogo_piston.set_value(1);
            mogo_clamped = true;
        }

        // Arm control
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && !digital_up_was_pressed) {
            get_arm_ready_to_score = false;
            if (arm_state == 0) {
                arm_motor.move_absolute(662, 100);
                arm_state++;
            } else if (arm_state == 1) {
                hook_intake.move_relative(-500, 200);
                arm_motor.move_absolute(2470, 50);
                arm_state++;
            } 
            digital_up_was_pressed = true;
        } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            digital_up_was_pressed = false;
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && !digital_down_was_pressed) {
           get_arm_ready_to_score = false;
            if (arm_state == 1) {
                arm_motor.move_absolute(0, 80);
                // hook_intake.move_absolute(last_target + 2995, 600);
                arm_state--;
            } else if (arm_state == 2) {
                arm_motor.move_absolute(662, 100);
                arm_state--;
            }
            digital_down_was_pressed = true;
        } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            digital_down_was_pressed = false;
        }
        // if (get_arm_ready_to_score) {
        //     if (arm_wait_ticks > 0) {
        //         arm_wait_ticks--;
        //     }
        //     if (arm_wait_ticks == 0) {
        //         arm_motor.move_absolute(2470, 80);
        //         arm_state++;
        //         arm_wait_ticks = 10;
        //         get_arm_ready_to_score = false;
        //     }
        // }



        pros::delay(20);
	}
}