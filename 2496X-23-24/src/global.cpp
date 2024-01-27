#include "global.h"
using namespace pros;


namespace glb {

    Motor FR(P_FR, E_MOTOR_GEAR_600, false);
    Motor FL(P_FL, E_MOTOR_GEAR_600, true);
    Motor MR(P_MR, E_MOTOR_GEAR_600, false);
    Motor ML(P_ML, E_MOTOR_GEAR_600, true);
    Motor BR(P_BR, E_MOTOR_GEAR_600, false);
    Motor BL(P_BL, E_MOTOR_GEAR_600, true);

    Motor intake(P_intake, E_MOTOR_GEAR_600, true);

    Imu inertial(P_imu);

    Motor_Group rightChassis({P_FR, P_MR, P_BR});
    Motor_Group leftChassis({P_FL, P_ML, P_BL});
    Motor_Group chassis({P_FR, P_FL, P_MR, P_ML, P_BR, P_BL});

    Controller con(E_CONTROLLER_MASTER);

}
