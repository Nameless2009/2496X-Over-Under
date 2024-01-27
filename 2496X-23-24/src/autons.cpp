#include "main.h"
#include "global.h"

using namespace pros;
using namespace glb;
using namespace std;


void drivePID(int desiredValue, int timeout=1500, string debug="off")
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

	// inertial.tare_heading();

	double initialValue = inertial.get_heading();
	if (initialValue > 180){
		initialValue = initialValue - 360;
	}


	if (abs(desiredValue) <= 1000){
		kP = 0.75;
		kI = 0.000575; 
		kD = 3.3;
	}
	else if (abs(desiredValue) <= 4000){
		kP = 0.275;
		kI = 0.0007;
		kD = 1.2489;
	}
	else {
		kP = 0.27;
		kI = 0.0007;
		kD = 1.248;
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

		if (debug == "off"){
			//do nothing
		}
		else if (debug == "debug"){
			con.clear();
			con.print(0,0, "error: %f", float(error));
		}


		prevError = error;

		if (abs(error) < 20)
		{
			count++;
		}

		if (count > 35)
		{
			enableDrivePID = false;
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

	double kP = 7;
	double kI = 0.0000000001; 
	double kD = 30;

	double maxI = 500;

	int time = 0;
	
	int integralThreshold = 30;


	chassis.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

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
			con.clear();
			con.print(0,0, "error: %f", float(error));
		}

		prevError = error;

		if (abs(error) < 5)
		{
			count++;
		}

		if (count > 30)
		{
			enableTurnPID = false;
		}

		delay(20);
		time = time + 20;
	}

	chassis.move(0);
}



void offSide()
{
	
}

void autonSkills()
{
	
}

void onSide()
{
	
}

void skipAutonomous()
{

}