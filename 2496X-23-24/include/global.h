#ifndef __GLOBAL__
#define __GLOBAL__ // start off c++ header files with this


#include "main.h"


using namespace pros;


namespace glb {

    #define P_FL 1
    #define P_FR 2
    #define P_ML 3
    #define P_MR 4
    #define P_BL 5
    #define P_BR 6
    #define P_imu 7
    #define P_intake 8 


    extern Motor FR;
    extern Motor FL;
    extern Motor MR;
    extern Motor ML;
    extern Motor BR;
    extern Motor BL;

    extern Motor intake;

    extern Imu inertial;

    extern Motor_Group rightChassis;
    extern Motor_Group leftChassis;
    extern Motor_Group chassis;

    extern Controller con;

}

#endif
