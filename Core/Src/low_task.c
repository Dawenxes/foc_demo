//
// Created by begre on 2024-06-04.
//
/**********************************

**********************************/
#include "low_task.h"


uint16_t hz_100_cnt = 0;
enum FOC_STATE foc_state = MOTOR_STOP;

void motor_start(void) {
    foc_algorithm_initialize();
    Speed_Ref = 20.0F;
    speed_close_loop_flag = 0;
    Iq_ref = 0.0f;

    hall_angle_add = 0.0005f;
    hall_speed = 0.0f;
    set_bldcm_enable();
}

void motor_stop(void) {
    set_bldcm_disable();
}



