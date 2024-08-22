#ifndef __AUTONS__
#define __AUTONS__ // start off c++ header files with this


#include "main.h"
#include "global.h"


using namespace pros;
using namespace std;

void drivePID(int desiredValue, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0, string debug="off");

void turnPID(int desiredValue, int timeout=1500, string turnType="point", string debug="off");

float calculatePID(float error);

void rightArc(double radius, double centralDegreeTheta, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0);

void leftArc(double radius, double centralDegreeTheta, int timeout=1500, string createTask="off", int taskStart=0, int taskEnd=0);

void farSideRush();

void farSideNoRush();

void closeSide();

void autonSkills();

void skipAutonomous();

void skillsMacro();


#endif