#ifndef __GLOBAL__
#define __GLOBAL__ // start off c++ header files with this


#include "main.h"


using namespace pros;


namespace glb {

    #define P_FL 4
    #define P_FR 6
    #define P_ML 5
    #define P_MR 7
    #define P_BL 8
    #define P_BR 9
    #define P_imu 21
    #define P_intake 3 
    #define P_slapper 10


    extern Motor FR;
    extern Motor FL;
    extern Motor MR;
    extern Motor ML;
    extern Motor BR;
    extern Motor BL;

    extern Motor intake;

    extern Motor slapper;

    extern ADIDigitalOut frontLeftWing;
    extern ADIDigitalOut frontRightWing;
    extern ADIDigitalOut backLeftWing;
    extern ADIDigitalOut backRightWing;

    extern ADIDigitalOut pto;
    extern ADIDigitalOut hangPiston;

    extern Imu inertial;

    extern Motor_Group rightChassis;
    extern Motor_Group leftChassis;
    extern Motor_Group chassis;

    extern Controller con;

}

#endif
