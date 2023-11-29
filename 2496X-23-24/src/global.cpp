#include "global.h"


namespace glb {
    // #define P_L_cata 8
    // #define P_R_cata 15
    // #define P_FL_chassis 1
    // #define P_BR_chassis 2
    // #define P_BL_chassis 3
    // #define P_FR_chassis 5
    // #define P_intake 9
    // #define P_catalimit 8
    // #define P_imu 21;
    // #define P_intakeLifter 1
    // #define P_wings 2
    // #define P_blocker_left 9
    // #define P_blocker_right 10


    // objects


    Motor cataL(P_L_cata, E_MOTOR_GEARSET_18, false); // 18 is for speed (normal/green), 36 is for torque (red), 06 is for turbo (blue)
    Motor cataR(P_R_cata, E_MOTOR_GEARSET_18, true);
    Motor chassis_FR(P_FR_chassis, E_MOTOR_GEARSET_18, true);
    Motor chassis_FL(P_FL_chassis, E_MOTOR_GEARSET_18, false);
    Motor chassis_BR(P_BR_chassis, E_MOTOR_GEARSET_18, true);
    Motor chassis_BL(P_BL_chassis, E_MOTOR_GEARSET_18, false);
    Motor intake(P_intake, E_MOTOR_GEARSET_18, false);


    Motor_Group rightChassis({P_FR_chassis, P_BR_chassis});
    Motor_Group leftChassis({P_FL_chassis, P_BL_chassis});
    Motor_Group chassis({P_FR_chassis, P_BR_chassis, P_FL_chassis, P_BL_chassis}); 

    ADIDigitalOut zoneMech('F', false);
    ADIDigitalOut wings('G', true);

    Motor_Group cata({P_L_cata, P_R_cata});

    Motor blockerLeft(P_blocker_left, E_MOTOR_GEARSET_36, true);
    Motor blockerRight(P_blocker_right, E_MOTOR_GEARSET_36, false);

    Motor_Group blocker({P_blocker_left, P_blocker_right});


    Imu inertial(P_imu);


    ADIDigitalIn catalimit(P_catalimit);


    Controller con(E_CONTROLLER_MASTER); // controller


}
