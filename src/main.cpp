#include "main.h"
#include "catLib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftDrive({4, -3, -5});  // Creates a motor group with forward ports and reversed ports
pros::MotorGroup rightDrive({-7, 10, 8});  // Creates a motor group with forward ports and reversed ports

pros::Rotation verticalEncoder(-14);
pros::Rotation horizontalEncoder(1);
pros::Imu inertial1(2);

catlib::Drivetrain dt(
    &leftDrive,
    &rightDrive,
    catlib::omniWheel::OMNI_325,
    450
);

catlib::PIDConstants lateral_pid(
    6.5,
    0,
    30
);

catlib::PIDConstants angular_pid(
    2.5,
    0,
    25
);

catlib::TrackingWheel vertical_tracker(
    &verticalEncoder,
    2.75,
    0.876
);

catlib::TrackingWheel horizontal_tracker(
    &horizontalEncoder,
    2.75,
    -3.2455
);

catlib::OdomSensors sensors(
    &inertial1,
    &vertical_tracker,
    &vertical_tracker
);

catlib::Chassis cat (
    &dt,
    &lateral_pid,
    &angular_pid,
    &sensors,
    catlib::DriveType::SPLIT_ARCADE
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	cat.calibrate();
    //cat.initializeOdom();
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
void autonomous() {
    cat.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    cat.movePID(17.5, 2000);
    cat.turnToHeadingPID(90);
    pros::delay(500);
    cat.turnToHeadingPID(180, 1, true);
    cat.movePID(17.5, 2000);
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
    cat.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	while (true) {
		// Arcade control scheme
		int dir = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		leftDrive.move(dir + turn);                      // Sets left motor voltage
		rightDrive.move(dir - turn);                     // Sets right motor voltage

        //cat.track();

        controller.print(0, 0, "X:%.0lf Y:%.0lf T:%.0lf   ", cat.getPose()[0], cat.getPose()[1], cat.getPoseWithTheta()[2]);
		pros::delay(10);                               // Run for 10 ms then update
	}
}