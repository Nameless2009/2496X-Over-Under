#include "main.h"
#include "global.h"

using namespace pros;
using namespace glb;
using namespace std;


void drivePID(int desiredValue, int timeout=1500)
{
	bool enableDrivePID = true;
	int prevError = 0;
	double totalError = 0;
	int count = 0;

	double kP;
	double kI;
	double kD;

	bool generateConstants;

	double maxI = 500;
	
	int integralThreshold = 150;

	int time = 0;

	chassis_FR.tare_position();
	chassis_FL.tare_position();
	chassis_BR.tare_position();
	chassis_BL.tare_position();

	// inertial.tare_heading();

	//check if passed = off
	//if not off set k to passed and create variable to stop distance checking to assign k 
	//if off set variable to enable distance checking for k

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
		int FRpos = chassis_FR.get_position();
		int FLpos = chassis_FL.get_position();
		int BRpos = chassis_BR.get_position();
		int BLpos = chassis_BL.get_position();

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
		int currentValue = (FRpos + BRpos + FLpos + BLpos) / 4;

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

		
			// con.clear();
			// con.print(0,0, "error: %f", float(error));

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
	
	//elims offside
	drivePID(2500);
	turnPID(90);
	wings.set_value(true);
	drivePID(1500, 2000);
	wings.set_value(false);
	turnPID(0);
	drivePID(-1400);
	turnPID(90);
	drivePID(-2500);
	turnPID(0);
	intake.move(-127);
	delay(300);
	turnPID(180);
	drivePID(-700, 1000);
	
	
	//new offside:
	// inertial.set_heading(315);
	// zoneMech.set_value(true);
	// delay(100);
	// turnPID(-90);
	// turnPID(-30);
	// drivePID(900);
	// zoneMech.set_value(false);
	// turnPID(0);
	// intake.move(-127);
	// delay(500);
	// turnPID(180);
	// drivePID(-500, 1000);
	// drivePID(500);
	// turnPID(135);
	// drivePID(1500);
	// turnPID(90);
	// drivePID(1700);
	



	//old offside:
	// inertial.set_heading(315);
	// zoneMech.set_value(true);
	// delay(1000);
	// drivePID(-300);
	// turnPID(-90);
	// delay(500);
	// turnPID(90);
	// zoneMech.set_value(false);
	// blocker.move_relative(500, 127);
	// drivePID(2100);
	// intake.move(127);
	// drivePID(500);
	// delay(500);
	// turnPID(80);
	// intake.move(-127);
	// delay(500);
	// drivePID(500);
	// turnPID(-40);
	// intake.move(127);
	// drivePID(700);
	// delay(500);
	// turnPID(90);
	// intake.move(-127);
	// delay(50);
	// wings.set_value(true);
	// drivePID(2500);
}

void autonSkills()
{
		//auton skills:
	intake.move(-127);
	turnPID(-19.5);
	drivePID(100);
	cata.move(127);
	delay(30000);
	while(catalimit.get_value()==false){
		cata.move(100);
	}
	cata.move(0);
	turnPID(20);
	drivePID(-900); 
	turnPID(0);
	drivePID(-4000);
	turnPID(-45);
	drivePID(-1000);
	turnPID(-80);
	drivePID(-3000, 1300); //give massive distance and tell to timeout for ramming (u need to time the distance needed)
	drivePID(600);
	drivePID(-3000, 1300);
	turnPID(0);
	drivePID(2300);
	turnPID(90);
	drivePID(1000);
	turnPID(165);
	wings.set_value(true);
	drivePID(1500, 2000);
	wings.set_value(false);
	turnPID(-155);
	drivePID(-1700);
	turnPID(180);
	wings.set_value(true);
	drivePID(1700, 2000);
	drivePID(-1000);
}

void onSide()
{
	
	// new onside part 1 and 2:
	
	// intake.move(127);
	// drivePID(500);
	// turnPID(180);
	// drivePID(1700);
	// turnPID(135);
	// drivePID(1200);
	// turnPID(90);
	// intake.move(-127);
	// drivePID(1000, 1000);
	// drivePID(-1000);

	// turnPID(0);
	// drivePID(1200);
	// turnPID(20);
	// intake.move(127);
	// drivePID(1350);
	// turnPID(140);
	// intake.move(-127);
	// delay(300);
	// drivePID(500);
	// turnPID(50);
	// intake.move(127);
	// drivePID(1000);
	// turnPID(180);
	// intake.move(-127);
	// wings.set_value(true);
	// drivePID(2000);
	

	//time saving auton
	// intake.move(127);
	// drivePID(500);
	// turnPID(180);
	// drivePID(1700);
	// turnPID(90);
	// drivePID(1200);
	// turnPID(180);
	// intake.move(-127);
	// delay(500);
	// turnPID(-15);
	// intake.move(127);
	// drivePID(1000);
	// turnPID(165);
	// drivePID(500);
	// intake.move(-127);
	// delay(500);
	// turnPID(30);
	// intake.move(127);
	// drivePID(1000);
	// turnPID(180);
	// intake.move(-127);
	// delay(100);
	// wings.set_value(true);
	// drivePID(3000);
	//delay(100);
	// drivePID(-1000);
	
	
	
	
	
	
	
	//old onside:

	// drivePID(2500);
	// turnPID(80);
	// intake.move(-127);
	// delay(1000);
	// drivePID(-500);
	// turnPID(-113);
	// intake.move(127);
	// drivePID(400);
	// delay(500);
	// turnPID(80);
	// intake.move(-127);
	// delay(500);
	// drivePID(500);
	// turnPID(-40);
	// intake.move(127);
	// drivePID(700);
	// delay(500);
	// turnPID(90);
	// intake.move(-127);
	// delay(50);
	// wings.set_value(true);
	// blocker.move_relative(1500, 127);
	// drivePID(2500, 500);
	// drivePID(-1000);
	// turnPID(0);
}

void skipAutonomous()
{
	drivePID(2500);
	turnPID(80);
	intake.move(-127);
	delay(1000);
	drivePID(-500);
	turnPID(-113);
	intake.move(127);
	drivePID(400);
	delay(500);
	turnPID(80);
	intake.move(-127);
	delay(500);
	drivePID(500);
	turnPID(-40);
	intake.move(127);
	drivePID(700);
	delay(500);
	turnPID(90);
	intake.move(-127);
	delay(50);
	wings.set_value(true);
	blocker.move_relative(1500, 127);
	drivePID(2500, 500);
	drivePID(-1000);
	turnPID(0);
}