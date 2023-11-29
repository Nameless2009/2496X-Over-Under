#ifndef __AUTONS__
#define __AUTONS__ // start off c++ header files with this


#include "main.h"
#include "global.h"


using namespace pros;

void drivePID(int desiredValue, int timeout=1500);

void turnPID(int desiredValue, int timeout=1500);

void offSide();

void onSide();

void autonSkills();

void skipAutonomous();



#endif