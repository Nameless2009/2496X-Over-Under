#include "main.h"
#include "global.h"
#include <cmath>
#include <limits>

using namespace pros;
using namespace glb;
using namespace std;

constexpr double gear_ratio = ((double)36/45);
constexpr double wheel_radius = 1.375;
constexpr double wheel_circumference = 2* M_PI * wheel_radius;
constexpr double start_heading = 0;


class Point {
	public:
		float x;
		float y;

		float theta = numeric_limits<float>::quiet_NaN(); //if theta is not a real number, quiet the errors


		//Constructor
		Point(float x, float y, float theta = numeric_limits<float>::quiet_NaN())
		:	x(x),
			y(y),
			theta(theta) {}


		//function that returns the distance between two points
		float distanceTo(const Point& other) const{
			float deltaX = x - other.x;
			float deltaY = y - other.y;
			return sqrt(deltaX * deltaX + deltaY * deltaY);
			// distance formula ^
		}

		//function that converts degrees to radians (multiply by pi and divide by 180)
		float degreesToRadians(float degrees){
			return degrees * M_PI / 180.0;
		}

		//function that calculates the angular error
		float angleError(const Point& other) {
			return other.theta - theta;
		}
};

void drivePID(int desiredValue, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0, string debug="off")
{
	bool enableDrivePID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;

	double kP;
	double kI;
	double kD;

	double maxI = 500;
	
	int integralThreshold = 150;

	int time = 0;

	chassis.tare_position();

	inertial.tare_heading();

	double initialValue = inertial.get_heading();
	if (initialValue > 180){
		initialValue = initialValue - 360;
	}


	if (abs(desiredValue) <= 1000){
		kP = 0.41;
		kI = 0.0007;
		kD = 1.27;
	}
	else if (abs(desiredValue) <= 4000){
		kP = 0.57;
		kI = 0.0007;
		kD = 1.27;
	}
	else {
		kP = 0.76;
		kI = 0.0007;
		kD = 1.27;
	}

	bool taskStarted = false;
	bool taskEnded = false;


	while (enableDrivePID)
	{

		if (time > timeout){
			enableDrivePID = false;
		}

		// get position of all motors:
		int FRpos = FR.get_position();
		int FLpos = FL.get_position();
		int BRpos = BR.get_position();
		int BLpos = BL.get_position();
		int MRpos = MR.get_position();
		int MLpos = ML.get_position();

		double currentIMUValue = inertial.get_heading();
		if(initialValue > 120){
			currentIMUValue = inertial.get_heading();
		} else if (initialValue < -120){
			currentIMUValue = inertial.get_heading() - 360;
		} else {
   			currentIMUValue = inertial.get_heading();
   			if(currentIMUValue > 180){
      			currentIMUValue = currentIMUValue - 360;
			} 
		}
		double headingCorrection = initialValue - currentIMUValue;
		headingCorrection = headingCorrection * 5;

		// get avg of motors:
		int currentValue = (FRpos + BRpos + FLpos + BLpos + MRpos + MLpos) / 6;

		// proportional
		double error = desiredValue - currentValue;

		// derivative
		int derivative = error - prevError;

		// integral
		if (abs(error) < integralThreshold)
		{
			totalError += error;
		}

		if (error > 0){
			totalError = min(totalError, maxI);
		}
		else{
			totalError = max(totalError, -maxI);
		}

		double speed = (error * kP + derivative * kD + totalError * kI);


		if (speed>127){
			speed = 127;
		}
		else if (speed < -127){
			speed = -127;
		}

		leftChassis.move(speed - headingCorrection);
		rightChassis.move(speed + headingCorrection);


		prevError = error;

		if (abs(error) < 30)
		{
			count++;
		}

		if (desiredValue <= 1000){
			if (count > 8)
			{
				enableDrivePID = false;
			}
		}
		else if (desiredValue <= 2000){
			if (count > 8)
			{
				enableDrivePID = false;
			}
		}
		else {
			if (count > 8){
				enableDrivePID = false;
			}
		}

		if (time >= taskStart && taskStarted == false){
			if (createTask == "frontWings"){
				frontLeftWing.set_value(true);
				frontRightWing.set_value(true);
				taskStarted == true;
			}
			else if (createTask == "backWings"){
				backLeftWing.set_value(true);
				backRightWing.set_value(true);
				taskStarted == true;
			}
			else if (createTask == "reverseIntake"){
				intake.move(127);
				taskStarted == true;
			}
			else if (createTask == "forwardIntake"){
				intake.move(-127);
				taskStarted == true;
			}
		}
		if (time >= taskEnd && taskEnded == false){
			if (createTask == "frontWings"){
				frontLeftWing.set_value(false);
				frontRightWing.set_value(false);
				taskEnded == true;
			}
			else if (createTask == "backWings"){
				backLeftWing.set_value(false);
				backRightWing.set_value(false);
				taskEnded == true;
			}
			else if (createTask == "reverseIntake"){
				intake.move(0);
				taskEnded == true;
			}
			else if (createTask == "forwardIntake"){
				intake.move(0);
				taskEnded == true;
			}
		}
		

		time = time + 20; //add one to time every cycle

		delay(20);
		
	}

	chassis.move(0);
}

void turnPID(int desiredValue, int timeout=1500, string turnType="point", string debug="off")
{
	bool enableTurnPID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;
	double position;
	double turnV;

	double kP = 3.6;
	double kI = 0.0000001;
	double kD = 13.4;

	double maxI = 500;

	int time = 0;
	
	int integralThreshold = 30;


	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

	if (debug == "debug"){
		con.clear();
	}
	

	//inertial.tare_heading();


	position = inertial.get_heading();
	if (position > 180){
		position = ((360-position) * -1);
	}

	if ((desiredValue < 0) && (position > 0)){
		if ((position - desiredValue) >= 180){
			desiredValue = desiredValue + 360;
			position = inertial.get_heading();
			turnV = (position + desiredValue);
		}
		else {
			turnV = (abs(position) + abs(desiredValue));
		}
	}
	else if ((desiredValue > 0) && (position < 0)) {
		if ((desiredValue - position) >= 180){
			position = inertial.get_heading();
			turnV = 360 - desiredValue - abs(position);
		}
		else {
			turnV = (position + desiredValue);
		}
	}

	while (enableTurnPID)
	{
		if (time > timeout){
			enableTurnPID = false;
		}

		// get avg of motors:
		position = inertial.get_heading();
		if (position > 180){ //make only > if not working
			position = ((360-position) * -1);
		}

		if ((desiredValue < 0) && (position > 0)){
			if ((position - desiredValue) >= 180){
				desiredValue = desiredValue + 360;
				position = inertial.get_heading();
				turnV = (position + desiredValue);
			}
			else {
				turnV = (abs(position) + abs(desiredValue));
			}
		}
		else if ((desiredValue > 0) && (position < 0)) {
			if ((desiredValue - position) >= 180){
				position = inertial.get_heading();
				turnV = 360 - desiredValue - abs(position);
			}
			else {
				turnV = (position + desiredValue); //for different constants for pid
			}
		}

		// proportional
		int error = desiredValue - position;

		// derivative
		int derivative = error - prevError;

		// integral
		if (abs(error) < integralThreshold)
		{
			totalError += error;
		}

		if (error > 0){
			totalError = min(totalError, maxI);
		}
		else{
			totalError = max(totalError, -maxI);
		}


		double speed = (error * kP + derivative * kD + totalError * kI);
		if (turnType == "point"){
			rightChassis.move(speed);
			leftChassis.move(-speed);
		}
		else if (turnType == "rightSwing"){
			rightChassis.move(speed);
		}
		else if (turnType == "leftSwing"){
			leftChassis.move(-speed);
		}

		if (debug == "off"){
			//do nothing
		}
		else if (debug == "debug"){
			con.print(0,0, "error: %f", float(error));
		}

		prevError = error;

		if (abs(error) < 5)
		{
			count++;
		}

		if (count > 10)
		{
			enableTurnPID = false;
		}

		delay(20);
		time = time + 20;
	}

	chassis.move(0);
}

float totalError;
float prevError;
float kP;
float kI;
float kD;
int integralThreshold = 30;
double maxI = 500;

float calculatePID(float error){

	if (abs(error) <= 1000){
		kP = 0.75;
		kI = 0.000575; 
		kD = 3.3;
	}
	else if (abs(error) <= 4000){
		kP = 0.275;
		kI = 0.0007;
		kD = 1.2489;
	}
	else {
		kP = 0.27;
		kI = 0.0007;
		kD = 1.248;
	}
	
	// calculate integral
	if (abs(error) < integralThreshold)
	{
		totalError += error;
	}

    // calculate derivative
    float derivative = error - prevError;
    prevError = error;

    // calculate output
    double speed = (error * kP) + (totalError * kI) + (derivative * kD);

	if (speed > 127){
		speed = 127;
	}
	else if (speed < -127){
		speed = -127;
	}

	return speed;

}

void leftArc(double radius, double centralDegreeTheta, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0){

	//central degree is in radians

	double centerArc = radius*centralDegreeTheta; 

	double rightArc = (radius - 265)*centralDegreeTheta;
	double leftArc = (radius + 265)*centralDegreeTheta;

	// make 5 inches into encoder ticks and make it so that one side is added 10 or so and the other side is added 0, so the arc is for the left side, not the center of the bot

	double speedProp = rightArc/leftArc;

	chassis.tare_position();
	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

	int count =0;
	int time =0;

	bool taskStarted = false;
	bool taskEnded = false;

	int heading = inertial.get_heading();

	while(1){
		
		if (time > timeout){
			break;
		}

		int FRpos = FR.get_position();
		int FLpos = FL.get_position();
		int BRpos = BR.get_position();
		int BLpos = BL.get_position();
		int MRpos = MR.get_position();
		int MLpos = ML.get_position();

		int currentLeftPosition = (FLpos + BLpos + MLpos)/3;
		int error = leftArc - currentLeftPosition;

		double leftcorrect = (currentLeftPosition * 360) / (2*M_PI*(-radius));

		if(leftcorrect > 120){
			heading = inertial.get_heading();
		} else if (leftcorrect < -120){
			heading = inertial.get_heading() - 360;
		} else {
   			heading = inertial.get_heading();
   			if(heading > 180){
      			heading = heading - 360;
			} 
		}

		int fix = int(heading + leftcorrect);
		fix = fix*5;

		leftChassis.move((calculatePID(error)) - fix);
		rightChassis.move(((calculatePID(error))*speedProp) + fix);

		if (abs(error) < 10)
		{
			count++;
		}

		if (count > 100){
			break;
		}

		if (time >= taskStart && taskStarted == false){
			if (createTask == "frontWings"){
				frontLeftWing.set_value(true);
				frontRightWing.set_value(true);
				taskStarted == true;
			}
			else if (createTask == "backWings"){
				backLeftWing.set_value(true);
				backRightWing.set_value(true);
				taskStarted == true;
			}
			else if (createTask == "reverseIntake"){
				intake.move(127);
				taskStarted == true;
			}
			else if (createTask == "forwardIntake"){
				intake.move(-127);
				taskStarted == true;
			}
		}
		if (time >= taskEnd && taskEnded == false){
			if (createTask == "frontWings"){
				frontLeftWing.set_value(false);
				frontRightWing.set_value(false);
				taskEnded == true;
			}
			else if (createTask == "backWings"){
				backLeftWing.set_value(false);
				backRightWing.set_value(false);
				taskEnded == true;
			}
			else if (createTask == "reverseIntake"){
				intake.move(0);
				taskEnded == true;
			}
			else if (createTask == "forwardIntake"){
				intake.move(0);
				taskEnded == true;
			}
		}		

		delay(20);
		time = time+20;

	}
	chassis.move(0);
}

void rightArc(double radius, double centralDegreeTheta, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0){

	
	//central degree is in radians

	double centerArc = radius*centralDegreeTheta; 

	double rightArc = (radius + 265)*centralDegreeTheta;
	double leftArc = (radius - 265)*centralDegreeTheta;

	double speedProp = leftArc/rightArc;

	chassis.tare_position();
	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

	int count =0;
	int time =0;

	int heading = inertial.get_heading();

	bool taskStarted = false;
	bool taskEnded = false;

	while(1){
		
		if (time > timeout){
			break;
		}

		int FRpos = FR.get_position();
		int FLpos = FL.get_position();
		int BRpos = BR.get_position();
		int BLpos = BL.get_position();
		int MRpos = MR.get_position();
		int MLpos = ML.get_position();

		int currentRightPosition = (FRpos + BRpos + MRpos)/3;
		int currentLeftPosition = (FLpos + BLpos + MLpos)/3;
		int error = rightArc - currentRightPosition;

		double rightcorrect = (currentRightPosition * 360) / (2*M_PI*(-radius));

		if(rightcorrect > 120){
			heading = inertial.get_heading();
		} else if (rightcorrect < -120){
			heading = inertial.get_heading() - 360;
		} else {
   			heading = inertial.get_heading();
   			if(heading > 180){
      			heading = heading - 360;
			} 
		}

		int fix = int(heading + rightcorrect);
		fix = fix*5;

		rightChassis.move((calculatePID(error)) + fix);
		leftChassis.move(((calculatePID(error))*speedProp) - fix);

		if (abs(error) < 30)
		{
			count++;
		}

		if (count > 3){
			break;
		}

		if (time >= taskStart && taskStarted == false){
			if (createTask == "frontWings"){
				frontLeftWing.set_value(true);
				frontRightWing.set_value(true);
				taskStarted == true;
			}
			else if (createTask == "backWings"){
				backLeftWing.set_value(true);
				backRightWing.set_value(true);
				taskStarted == true;
			}
			else if (createTask == "reverseIntake"){
				intake.move(127);
				taskStarted == true;
			}
			else if (createTask == "forwardIntake"){
				intake.move(-127);
				taskStarted == true;
			}
		}
		if (time >= taskEnd && taskEnded == false){
			if (createTask == "frontWings"){
				frontLeftWing.set_value(false);
				frontRightWing.set_value(false);
				taskEnded == true;
			}
			else if (createTask == "backWings"){
				backLeftWing.set_value(false);
				backRightWing.set_value(false);
				taskEnded == true;
			}
			else if (createTask == "reverseIntake"){
				intake.move(0);
				taskEnded == true;
			}
			else if (createTask == "forwardIntake"){
				intake.move(0);
				taskEnded == true;
			}
		}

		delay(20);
		time = time+20;

	}
	chassis.move(0);
}

// void boomerang(float x, float y, float theta, float dlead){

// 	float integral;
	
// 	float linearError;
// 	float linearPower;
// 	float angularError;
// 	float angularPower;

// 	bool angularSettled;
// 	bool linearSettled;

// 	double x = 0;
// 	double y = 0;

// 	chassis.tare_position();

// 	Point calculations(0,0); //do not use functions that require x and y with this object!!!

// 	//calculate target point in standard form
// 	float radiansTheta = calculations.degreesToRadians(theta);
// 	Point target(x, y, M_PI_2 - radiansTheta);

// 	while (!angularSettled && !linearSettled){

// 		double heading = fmod((360 - inertial.get_heading()) + start_heading, 360);
// 		// the float remainder of parameter 1 divided by parameter 2

// 		double FRpos = FR.get_position();
// 		double FLpos = FL.get_position();
// 		double BRpos = BR.get_position();
// 		double BLpos = BL.get_position();
// 		double MRpos = MR.get_position();
// 		double MLpos = ML.get_position();

// 		double average_encoder_position = (FRpos + MRpos + BRpos + FLpos + MLpos + BLpos)/6; 
		
// 		double distance_travelled = (average_encoder_position / 360) * wheel_circumference * gear_ratio;
		
// 		Point robot(/*robot x value*/, /*robot x value*/);
		
// 		float d = robot.distanceTo(target);
		
// 		Point carrot(target.x - d*cos(theta) * dlead, target.y - d*sin(theta)*dlead);



// 		linearError = robot.distanceTo(target);
// 		linearPower = linear //update pid values (use calculation function)
// 		angularError = robot.angleError(theta);
// 		//same for angular power


// 		delay(10);
// 	}
// }

void offSide()
{
	
}

void autonSkills()
{
	
}

void onSide()
{
	
}

void openWingsHalfway(void*){
	delay(1000);
	intake.move(-127);
	backLeftWing.set_value(true);
	backRightWing.set_value(true);
}

void skipAutonomous()
{
	//onside
	// intake.move(-127); //intake is reversed for some reason
	// drivePID(3000);
	// turnPID(-60);
	// inertial.set_heading(0);
	// drivePID(-1500);
	// drivePID(400);
	// turnPID(-90);
	// drivePID(2500);
	// turnPID(-100);
	// intake.move(127);
	// delay(150);
	// turnPID(90);
	// intake.move(-127);
	// drivePID(1300);
	// turnPID(3);
	// drivePID(-1500);
	// rightArc(10, -500);
	// rightArc(7, 700);
	// intake.move(127);
	
	//6ball
	intake.move(-127);
	drivePID(2500);
	drivePID(-2800);
	turnPID(60);
	intake.move(127);
	delay(300);
	turnPID(-76);
	inertial.tare();
	intake.move(-127);
	drivePID(1270);
	drivePID(-1280);
	backLeftWing.set_value(true);
	rightArc(900, -4);
	drivePID(500);
	turnPID(180);
	intake.move(127);
	delay(200);
	drivePID(1000, 500);
	drivePID(-800);
	turnPID(-70);
	drivePID(2000);

	//prog skills
	// rightArc(1500, -1);
	// rightArc(350, 2.19);
	// drivePID(-200);
	// //slapper.move(127);
	// delay(1000);
	// rightArc(1500, 1.8);
	// frontLeftWing.set_value(true);
	// frontRightWing.set_value(true);
	// intake.move(-127);
	// drivePID(2000);
	// drivePID(-1000);
	// drivePID(1000);
	// drivePID(-500);
	// turnPID(80);
	// drivePID(1200);
	// leftArc(295, 2.78);
	// drivePID(3500);


}