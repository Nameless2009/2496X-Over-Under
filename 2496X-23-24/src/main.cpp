#include "main.h"
#include "global.h"
#include "autons.h"

using namespace glb;
using namespace pros;
using namespace std;

bool slapperToggle = false;
int hangSequence = 0;

bool FLwing = false;
bool FRwing = false;
bool BRwing = false;
bool BLwing = false;

void chassisCode(){
	double rightstick = con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
	double leftstick = con.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

	if (rightstick > 0 && leftstick < 0){
		rightstick = 0.85*rightstick;
		leftstick = 0.85*leftstick;
	}
	else if (rightstick < 0 && leftstick > 0){
		rightstick = 0.85*rightstick;
		leftstick = 0.85*leftstick;
	}

	rightChassis.move(rightstick);
	leftChassis.move(leftstick);
}

void intakeCode(){
	if (con.get_digital(E_CONTROLLER_DIGITAL_R1)){
		intake.move(127);
	}
	else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)){
		intake.move(-127);
	}
	else{
		intake.move(0);
	}
}

void slapperCode(){
	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
		slapperToggle = !slapperToggle;
	}

	if (slapperToggle) {
		slapper.move(95);
	}
	else {
		slapper.move(0);
	}
}

void wingsCode(){
	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
		BRwing = !BRwing;
	}
	if (BRwing == true) {
		backRightWing.set_value(true);
	}
	else {
		backRightWing.set_value(false);
	}


	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)){
		BLwing = !BLwing;
	}
	if (BLwing == true) {
		backLeftWing.set_value(true);
	}
	else {
		backLeftWing.set_value(false);
	}


	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
		FRwing = !FRwing;
	}
	if (FRwing == true) {
		frontRightWing.set_value(true);
	}
	else {
		frontRightWing.set_value(false);
	}


	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)){
		FLwing = !FLwing;
	}
	if (FLwing == true) {
		frontLeftWing.set_value(true);
	}
	else {
		frontLeftWing.set_value(false);
	}
}

void hangCode(){
	if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)){
		hangSequence++;
	}

	if (hangSequence == 1){
		hangPiston.set_value(true);
	}
	else if (hangSequence == 2){
		pto.set_value(false);
	}
	else if (hangSequence == 3){
		chassis.move(127);
		delay(1000);
		chassis.move(0);
	}
	else {
		hangSequence = 0;
	}
	
}


bool farsiderush = false;
bool farsidenorush = false;
bool closeside = false;
bool skillsAuton = false;
bool skipAuton = false;

bool rightclick = false;
bool leftclick = false;
int selection = 0;
bool centerclick = false;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	centerclick = !centerclick;
}
void on_left_button()
{
	//nothing for now
}
void on_right_button()
{
	rightclick = true;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	// pros::lcd::initialize();

	// pros::lcd::register_btn0_cb(on_left_button);
	// pros::lcd::register_btn1_cb(on_center_button);
	// pros::lcd::register_btn2_cb(on_right_button);

	// int counter = 0;

	// while (1){
	// 	if (rightclick == true){
	// 		selection++;
	// 		rightclick = false;
	// 	}
		
	// 	if (centerclick == true){
	// 		if (selection == 0){
	// 			skipAuton = true;
	// 		}
	// 		else if (selection == 1){
	// 			farsiderush = true;
	// 		}
	// 		else if (selection == 2){
	// 			farsidenorush = true;
	// 		}
	// 		else if (selection == 3){
	// 			closeside = true;
	// 		}
	// 		else if (selection == 4){
	// 			skillsAuton = true;
	// 		}

	// 		counter++; //timer start
	// 	}

	// 	if (selection == 0){
	// 		pros::lcd::clear_line(0);
	// 		pros::lcd::set_text(0, "Nothing Selected, Skip Auto");
	// 	}
	// 	else if (selection == 1){
	// 		pros::lcd::clear_line(0);
	// 		pros::lcd::set_text(0, "Far Side Rush Auto Selected");
	// 	}
	// 	else if (selection == 2){
	// 		pros::lcd::clear_line(0);
	// 		pros::lcd::set_text(0, "Far Side No Rush Auto Selected");
	// 	}
	// 	else if (selection == 3){
	// 		pros::lcd::clear_line(0);
	// 		pros::lcd::set_text(0, "Close Side Auto Selected");
	// 	}
	// 	else if (selection == 4){
	// 		pros::lcd::clear_line(0);
	// 		pros::lcd::set_text(0, "Skills Auto Selected");
	// 	}
	// 	else {
	// 		selection = 0;
	// 	}

	// 	delay(20);
	// 	if (counter >= 250){
	// 		break; //break after delay
	// 	}
	// }

	chassis.set_brake_modes(E_MOTOR_BRAKE_COAST);
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
	if (skillsAuton){
		autonSkills();
	}
	else if (farsiderush){
		farSideRush();
	}
	else if (farsidenorush){
		farSideNoRush();
	}
	else if (closeside){
		closeSide();
	}
	else { //if nothing was clicked
		skipAutonomous();      
	}
	
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
		
		chassisCode();
		intakeCode();
		slapperCode();
		wingsCode();
		hangCode();

	}
}