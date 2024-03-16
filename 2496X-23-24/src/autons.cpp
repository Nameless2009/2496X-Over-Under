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

void drivePID(int desiredValue, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0, int chainSpeed = 0, string debug="off")
{
	bool enableDrivePID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;

	double kP;
	double kI;
	double kD;

	bool chain;

	double maxI = 500;
	
	int integralThreshold = 150;

	int time = 0;

	chassis.tare_position();

	//inertial.tare_heading();

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

	if (chainSpeed == 0){
		chain = false;
	}
	else {
		chain = true;
	}


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

		if (chain == true && abs(speed) <= chainSpeed){
			enableDrivePID = false;
		}

		if (time >= taskStart && taskStarted == false){
			if (createTask == "frontWings"){
				frontLeftWing.set_value(true);
				frontRightWing.set_value(true);
				taskStarted = true;
			}
			else if (createTask == "backWings"){
				backLeftWing.set_value(true);
				backRightWing.set_value(true);
				taskStarted = true;
			}
			else if (createTask == "frontRightWing"){
				frontRightWing.set_value(true);
				taskStarted = true;
			}
			else if (createTask == "reverseIntake"){
				intake.move(127);
				taskStarted = true;
			}
			else if (createTask == "forwardIntake"){
				intake.move(-127);
				taskStarted = true;
			}
		}
		if (time >= taskEnd && taskEnded == false){
			if (createTask == "frontWings"){
				frontLeftWing.set_value(false);
				frontRightWing.set_value(false);
				taskEnded = true;
			}
			else if (createTask == "backWings"){
				backLeftWing.set_value(false);
				backRightWing.set_value(false);
				taskEnded = true;
			}
			else if (createTask == "frontRightWing"){
				frontRightWing.set_value(false);
				taskEnded = true;
			}
			else if (createTask == "reverseIntake"){
				intake.move(0);
				taskEnded = true;
			}
			else if (createTask == "forwardIntake"){
				intake.move(0);
				taskEnded = true;
			}
		}		

		delay(20);
		time = time + 20; //add one to time every cycle
		
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
		kI = 0.0007; //0.0007
		kD = 1.2489;
	}
	else {
		kP = 0.27;
		kI = 0.0007; //0.007
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

float totalError2;
float prevError2;
float kP2;
float kI2;
float kD2;
int integralThreshold2 = 30;
double maxI2 = 500;

float calculatePID2(float error){

	if (abs(error) <= 1000){
		kP2 = 0.75;
		kI2 = 0.000575; 
		kD2 = 3.3;
	}
	else if (abs(error) <= 4000){
		kP2 = 0.275;
		kI2 = 0.0007; //0.0007
		kD2 = 1.2489;
	}
	else {
		kP2 = 0.27;
		kI2 = 0.0007; //0.007
		kD2 = 1.248;
	}
	
	// calculate integral
	if (abs(error) < integralThreshold2)
	{
		totalError2 += error;
	}

    // calculate derivative
    float derivative = error - prevError2;
    prevError2 = error;

    // calculate output
    double speed = (error * kP2) + (totalError2 * kI2) + (derivative * kD2);

	if (speed > 127){
		speed = 127;
	}
	else if (speed < -127){
		speed = -127;
	}

	return speed;

}


void rightArc(double radius, double centralDegreeTheta, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0, int chainSpeed=0){

	double rightArc = (centralDegreeTheta / 360)*2*M_PI*(radius + 530);
	double leftArc = (centralDegreeTheta / 360)*2*M_PI*(radius);

	bool chain = true;

	//double speedProp = rightArc/leftArc;

	chassis.tare_position();
	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

	int count =0;
	int time =0;

	bool taskStarted = false;
	bool taskEnded = false;

	double init_heading = inertial.get_heading(); 
	if (init_heading > 180){
		init_heading = init_heading - 360;
	}

	if (chainSpeed == 0){
		chain = false;
	}
	else {
		chain = true;
	}

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

		int right_error = rightArc - currentRightPosition;
		int left_error = leftArc - currentLeftPosition;

		double leftcorrect = (currentLeftPosition * 360) / (2*M_PI*(radius)); 

		int heading = inertial.get_heading() - init_heading; 
		if(centralDegreeTheta > 0){ 
			if(heading > 300){
				heading = heading - 360; 
			}
		} else {
			if( heading > 30){ 
   				heading = heading - 360; 
			}
		}

		int fix = int(heading - leftcorrect);
		fix = fix*5;
		leftChassis.move(calculatePID(left_error) + fix);
		rightChassis.move(calculatePID2(right_error) - fix);

		if ((abs(leftArc - currentLeftPosition) <= 20) && (abs(rightArc - currentRightPosition) <= 20)){ 
			count++;
		}
		if (count >= 2 || time > timeout){
			break;
		}

		if (chain == true && abs(calculatePID(left_error)) <= chainSpeed && abs(calculatePID2(right_error)) <= chainSpeed){
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

void leftArc(double radius, double centralDegreeTheta, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0, int chainSpeed=0){


	double rightArc = (centralDegreeTheta / 360)*2*M_PI*(radius);
	double leftArc = (centralDegreeTheta / 360)*2*M_PI*(radius + 530);

	bool chain;

	//double speedProp = leftArc/rightArc;

	chassis.tare_position();
	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

	int count =0;
	int time =0;

	
	double init_heading = inertial.get_heading(); 
	if (init_heading > 180){
		init_heading = init_heading - 360;
	}
	bool taskStarted = false;
	bool taskEnded = false;
	
	// con.clear();

	if (chainSpeed == 0){
		chain = false;
	}
	else{
		chain = true;
	}

	while(1){

		int FRpos = FR.get_position();
		int FLpos = FL.get_position();
		int BRpos = BR.get_position();
		int BLpos = BL.get_position();
		int MRpos = MR.get_position();
		int MLpos = ML.get_position();

		int currentRightPosition = (FRpos + BRpos + MRpos)/3;
		int currentLeftPosition = (FLpos + BLpos + MLpos)/3;
		int right_error = rightArc - currentRightPosition;
		int left_error = leftArc - currentLeftPosition;

		double rightcorrect = (currentRightPosition * 360) / (2*M_PI*(radius)); 

		//con.print(0,0, "imu: %f", float(inertial.get_heading()));

		int heading = inertial.get_heading() - init_heading; 
		if(centralDegreeTheta > 0){ 
			if(heading > 30){
				heading = heading - 360; 
			}
		} else {
			if( heading > 300){ 
   				heading = heading - 360; 
			}
		}

		int fix = int(heading + rightcorrect);
		fix = fix*5;
		leftChassis.move(calculatePID(left_error) + fix);
		rightChassis.move(calculatePID2(right_error) - fix);
		

		//con.print(0,0, "rc: %f", float(rightArc));

		if ((abs(leftArc - currentLeftPosition) <= 20) && (abs(rightArc - currentRightPosition) <= 20)){ 
			count++;
		}
		if (count >= 2 || time > timeout){
			break;
		}

		if (chain == true && abs(calculatePID(left_error)) <= chainSpeed && abs(calculatePID2(right_error)) <= chainSpeed){
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

void closeSide()
{
	//awp
	hangPiston.set_value(true);
	delay(50);
	hangPiston.set_value(false);
	inertial.set_heading(15);
	intake.move(-127);
	drivePID(2150);
	drivePID(-1200);
	leftArc(250, -120);
	backRightWing.set_value(true);
	delay(800);
	leftArc(200, 50);
	turnPID(75);
	delay(1000);
	turnPID(105);
	intake.move(127);
	backRightWing.set_value(false);
	drivePID(1700);
}

void autonSkills()
{
	//prog skills - derrae
	hangPiston.set_value(true);
	delay(50);
	hangPiston.set_value(false);
	inertial.set_heading(305);
	rightArc(1500, -58);
	rightArc(160, 122);
	drivePID(-330);
	backLeftWing.set_value(true);
	slapper.move(100);
	delay(24000);
	slapper.move(0);
	backLeftWing.set_value(false);
	intake.move(-127);
	drivePID(2200);
	// rightArc(2000, 70, 3000, "frontWings", 200, 30000);
	// frontLeftWing.set_value(true);
	// frontRightWing.set_value(true);
	turnPID(90, 15000);
	intake.move(127);
	frontLeftWing.set_value(true);
	frontRightWing.set_value(true);
	drivePID(3000);
	drivePID(-500);
	frontLeftWing.set_value(false);
	frontRightWing.set_value(false);
	turnPID(160);
	drivePID(1000, 1500, "off", 0, 0, 20);
	leftArc(150, 160);
	turnPID(0);
	intake.move(127);
	drivePID(1000);
	frontLeftWing.set_value(true);
	frontRightWing.set_value(true);
	drivePID(2400,3000, "off", 0, 0, 20); // short barrier starts here
	frontRightWing.set_value(false);
	leftArc(1100, 90);
	turnPID(-80);
	drivePID(500, 500);
	drivePID(-500);
	drivePID(800, 800);
	drivePID(-300);
	turnPID(0);
	leftArc(400, -90);
	drivePID(-400);
	frontLeftWing.set_value(false);
	frontRightWing.set_value(false);
	drivePID(500);
	turnPID(160);
	intake.move(-127);
	drivePID(1000);
	turnPID(-90);
	frontLeftWing.set_value(true);
	frontRightWing.set_value(true);
	intake.move(127);
	drivePID(1100);
	turnPID(180);
	frontLeftWing.set_value(false);
	frontRightWing.set_value(false);
	backRightWing.set_value(true);
	drivePID(-2000);
	backRightWing.set_value(false);
	drivePID(1500);
	turnPID(-90);
	drivePID(1100);
	turnPID(180);
	backLeftWing.set_value(true);
	drivePID(-2000);
	backLeftWing.set_value(false);
	drivePID(500);
	turnPID(-90);
	drivePID(2400);
	turnPID(0);
	rightArc(900,90);
	drivePID(2000, 700);
	drivePID(-500);
	drivePID(3000);
}

void farSideRush()
{
	//6ball
	frontRightWing.set_value(true);
	inertial.set_heading(348);
	hangPiston.set_value(true);
	delay(50);
	hangPiston.set_value(false);
	intake.move(-127);
	drivePID(2500, 2000, "frontRightWing", 0, 200);
	drivePID(-2700);
	turnPID(50);
	intake.move(127);
	delay(300);
	turnPID(-91);
	intake.move(-127);
	drivePID(1300);
	drivePID(-1350, 1500, "off", 0, 0, 20);
	backLeftWing.set_value(true);
	rightArc(700, -90);
	drivePID(-800, 700);
	drivePID(600);
	turnPID(0);
	intake.move(127);
	delay(80);
	drivePID(800, 600);
	drivePID(-800);
	turnPID(-70);
	intake.move(-127);
	drivePID(2100);
	turnPID(60);
	intake.move(127);
	drivePID(2500, 2000, "frontWings", 0, 10000);

}

void farSideNoRush()
{

}

void skipAutonomous()
{

}