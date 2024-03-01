#include "global.h"
using namespace pros;


namespace glb {

    Motor FR(P_FR, E_MOTOR_GEAR_600, true);
    Motor FL(P_FL, E_MOTOR_GEAR_600, false);
    Motor MR(P_MR, E_MOTOR_GEAR_600, true);
    Motor ML(P_ML, E_MOTOR_GEAR_600, false);
    Motor BR(P_BR, E_MOTOR_GEAR_600, true);
    Motor BL(P_BL, E_MOTOR_GEAR_600, false);

    Motor intake(P_intake, E_MOTOR_GEAR_600, true);
    
    Motor slapper(P_slapper, E_MOTOR_GEAR_200, true);

    ADIDigitalOut frontLeftWing('A', false);
    ADIDigitalOut frontRightWing('B', false);
    ADIDigitalOut backLeftWing('C', false);
    ADIDigitalOut backRightWing('D', false);

    ADIDigitalOut pto('E', true);
    ADIDigitalOut hangPiston('F', false);

    Imu inertial(P_imu);

    Motor_Group rightChassis({P_FR, P_MR, P_BR});
    Motor_Group leftChassis({P_FL, P_ML, P_BL});
    Motor_Group chassis({P_FR, P_FL, P_MR, P_ML, P_BR, P_BL});

    Controller con(E_CONTROLLER_MASTER);

}
