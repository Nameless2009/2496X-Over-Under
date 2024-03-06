#ifndef __AUTONS__
#define __AUTONS__ // start off c++ header files with this


#include "main.h"
#include "global.h"


using namespace pros;
using namespace std;

void drivePID(int desiredValue, int timeout=1500, string debug="off");

void turnPID(int desiredValue, int timeout=1500, string turnType="point", string debug="off");

float calculatePID(float error);

void rightArc(double radius, int centralDegreeTheta, int timeout=1500);

void leftArc(double radius, int centralDegreeTheta, int timeout=1500);

void offSide();

void onSide();

void autonSkills();

void skipAutonomous();


#endif