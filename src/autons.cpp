#include "main.h"
#include "pros/adi.hpp"
#include <cmath>
#define PI 3.141592653589793238462643383279502884197


/////
// For installation, upgrading, documentations and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// flywheel Motor
pros::Motor autonFlywheel(20, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);

// intake motors
// pros::Motor IntakeRight(10, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor autonIntake(17, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

// Expansion
pros::adi::DigitalOut autonWings('H');
pros::adi::DigitalOut autonDescore('C');
pros::adi::DigitalOut autonIntakePneumatic('G');  

// These are out of 127
const int DRIVE_SPEED = 110;  
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(3, 0, 20);
  chassis.pid_drive_constants_set(10, 0, 100);
  chassis.pid_turn_constants_set(3, 0, 20);
  chassis.pid_swing_constants_set(5, 0, 30);

  chassis.pid_turn_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_swing_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_drive_exit_condition_set(300_ms, 1_in, 500_ms, 3_in, 750_ms, 750_ms);

  chassis.slew_drive_constants_set(7_in, 80);
}

double targetAngle = 0;
void turn_relative(double theta_deg, int speed){
  targetAngle += theta_deg;
  chassis.pid_turn_set(targetAngle, speed);
}


const double DRIVETRAIN_WIDTH = 12.6; //TODO: MEASURE DRIVETRAIN WIDTH AND SET HERE FOR CURVE CALCULATIONS
double sqr(double x){
  return x * x;
}

double derivative(double r, double x, double xc) {
  double dx = (0.5) * (1.0/sqrt((sqr(r) - sqr(x - xc)))) * (-2.0) * (x - xc);
  //// return (dx >= -1000000) ? dx : -1000000;
  return dx;
}

void Curve(double x1, double x2, double x3, double y1, double y2, double y3) {
  double n = (-x3 + x1);
  double m = (-x2 + x1);

  double yc = (n * sqr(x2) - n * sqr(x1) - m * sqr(x3) + m * sqr(x1) - m * sqr(y3) + m * sqr(y1) + n * sqr(y2) - n * sqr(y1)) / (-2 * (-y2 + y1) * n + 2 * m * (-y3 + y1));
  double xc = (-sqr(x3) + sqr(x1) - sqr(y3) + sqr(y1) - 2 * yc * (-y3 + y1)) / (2 * (-x3 + x1));

  double r = sqrt(sqr(x1 - xc) + sqr(y1 - yc));

  ////cout << "x: " << xc << " y: " << yc << " r: " << r << endl;

  double ch = sqrt(sqr(x3 - x1) + sqr(y3 - y1));

  ////cout << "Chord length: " << ch << endl;

  double ca = 2 * asin(ch / 2 / r);
  ////cout << "central angle: " << ca << endl;
  ////cout << "central angle deg: " << ca * 180 / PI << endl;

  double al = ca * r;
  ////cout << "short arc length: " << al << endl;

  double al2 = ca * (r + DRIVETRAIN_WIDTH);
  ////cout << "long arc length: " << al2 << endl;

  double t = al2 / 120;
  ////cout << "\"time\": " << t << endl;

  double v2 = al / t;
  ////cout << "slow side velocity: " << v2 << endl;

  double dx = derivative(r, x3, xc);
  double dxStart = derivative(r, x1, xc);
  ////cout << "dx: " << dx << endl;

  double thetaRel = atan(dx);
  double thetaRelStart = atan(dxStart);

  double thetaStart;
  if (x2 < xc) {
    thetaStart = 90.0 + thetaRel * 180 / PI;
  } else {
    thetaStart = 90.0 - thetaRel * 180 / PI;
  }

  double thetaFinal;
  if (x2 < xc) {
    thetaFinal = 90.0 - thetaRel * 180 / PI;
  } else {
    thetaFinal = 90.0 + thetaRel * 180 / PI;
  }
  ////cout << "final angle: " << thetaFinal << endl;

  if (x2 < xc) {
    chassis.pid_turn_set(thetaStart, TURN_SPEED);
    chassis.pid_swing_set(ez::LEFT_SWING, thetaFinal, 120, v2);
  } else {
    chassis.pid_turn_set(thetaStart, TURN_SPEED);
    chassis.pid_swing_set(ez::RIGHT_SWING, thetaFinal, 120, v2);
  }
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 30
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(30);  // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 30 speed
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 30
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(30);  // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 30 speed
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees.
// If interfered, robot will drive forward and then attempt to drive backwards.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .

void awp() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  turn_relative(90, 100);
}

void six_ball() {

}

void elims_close_side() {

}

void elims_six_ball() {

}

void skills() {

}