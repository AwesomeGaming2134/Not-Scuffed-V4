#include "main.h"

/////
// For installation, upgrading, documentations and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// flywheel Motor
pros::Motor Fly(3, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);

// intake motors
// pros::Motor IntakeRight(10, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor Intake(1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

// Expansion
pros::ADIDigitalOut Walls('H');
pros::ADIDigitalOut Hang('C');
pros::ADIDigitalOut IntakePneumatic('G');  

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER) ;

// Chassis constructor
ez::Drive chassis (
	// Left Chassis Ports (negative port will reverse it!)
	//   the first port is used as the sensor
	{-4, -7, -14}

	// Right Chassis Ports (negative port will reverse it!)
	//   the first port is used as the sensor
	,{6, 12, 16}

	// IMU Port
	,10

	// Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
	,2.75

	// Cartridge RPM
	,600

	// External Gear Ratio (MUST BE DECIMAL) This is WHEEL GEAR / MOTOR GEAR
	// eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 84/36 which is 2.333
	// eg. if your drive is 60:36 where the 36t is powered, your RATIO would be 60/36 which is 0.6
	// eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 36/60 which is 0.6
	,1.33333333
);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Print our branding over your terminal :D  
	pros::delay(500); // Stop the user from doing anything while legacy ports configure

	// Configure your chassis controls
	chassis.opcontrol_curve_buttons_toggle(true); // Enables modifying the controller curve with buttons on the joysticks
	chassis.opcontrol_drive_activebrake_set(0); // Sets the active brake kP. We recommend 0.1.
	chassis.opcontrol_curve_default_set(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
	default_constants(); // Set the drive to your own constants from autons.cpp!

	// These are already defaulted to these buttons, but you can change the left/right curve buttons here!
	// chassis.opcontrol_curve_buttons_left_set (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
	// chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

	// Autonomous Selector using LLEMU
	ez::as::auton_selector.autons_add({
		Auton("Example Drive\n\nDrive forward and come back.", drive_example),
		Auton("Example Turn\n\nTurn 3 times.", turn_example),
		Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
		Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
		Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
		Auton("Combine all 3 movements", combining_movements),
		Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
	});

	// Initialize chassis and auton selector
	chassis.initialize();
	ez::as::initialize();
	master.rumble(".");
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  	// . . .
}



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
  	// . . .
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
	chassis.pid_targets_reset(); // Resets PID targets to 0
	chassis.drive_imu_reset(); // Reset gyro position to 0
	chassis.drive_sensor_reset(); // Reset drive sensors to 0
	chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency

	ez::as::auton_selector.selected_auton_call(); // Calls selected auton from autonomous selector
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

bool intaketoggle = false;
bool shoottoggle = false;
bool verticalExpanded = false;
bool wallsExpanded = false;
bool liftExpanded = false;
double counter = 0;
bool flyReverse = false;
bool intakeallowed = false;
int power = 125;

void opcontrol() {
	// This is preference to what you like to drive on
	chassis.drive_brake_set(MOTOR_BRAKE_COAST);
    int timer = 0;
    int wallsTimer = 0;
    int verticalTimer = 0;
    int liftTimer = 0;
    int flyR = 0;
    int fly = 0;
    int pwr = 0;
    int ds = 0;
  
  	while (true) {
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) { 
		// Enable / Disable PID Tuner
		//  When enabled: 
		//  * use A and Y to increment / decrement the constants
		//  * use the arrow keys to navigate the constants
		if (master.get_digital_new_press(DIGITAL_X)) 
       		chassis.pid_tuner_toggle();
        
      	// Trigger the selected autonomous routine
      	if (master.get_digital_new_press(DIGITAL_B)) 
        	autonomous();

      	chassis.pid_tuner_iterate(); // Allow PID Tuner to iterate
    } 

        // intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            Intake = 127;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            Intake = -127;
        } else {
            Intake = 0;
        }

        // walls
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && wallsTimer <= timer) {
            wallsExpanded = !wallsExpanded;
            Walls.set_value(wallsExpanded);
            wallsTimer = timer + 50;
        }

        // vertical walls
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) && verticalExpanded <= timer) {
            verticalExpanded = !verticalExpanded;
            IntakePneumatic.set_value(verticalExpanded);
            verticalTimer = timer + 50;
        }

        // Fly wheel
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) && shoottoggle == false && fly <= timer) {
            Fly = flyReverse ? -power : power;

            shoottoggle = true;
            fly = timer + 50;
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) && shoottoggle == true && fly <= timer) {
            Fly = 0;

            shoottoggle = false;
            fly = timer + 50;
        }

        // Reverse flywheel
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) && flyR <= timer) {
            flyReverse = !flyReverse;
            flyR = timer + 50;
        }

        // flywheel power
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && pwr <= timer) {
            if (power < 126) {
                power = (int)power + 2;
            }
            pwr = timer + 10;
        }    

    // chassis.opcontrol_tank(); // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE); // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  	}
}
