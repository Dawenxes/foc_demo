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


void low_control_task(void) {
    if (foc_state == MOTOR_RUN) {
        motor_start();
        foc_state = MOTOR_RUNNING;
    } else {
        //motor_stop();
        foc_state = MOTOR_STOP;
    }

    if (key1_flag == 1) {
        if (foc_state == MOTOR_RUN) {
            foc_state = MOTOR_STOP;
        } else {
            foc_state = MOTOR_GET_OFFSET;
        }
        key1_flag = 0;
    }
    if (key2_flag == 1) {
        Speed_Ref += 5.0f;
        key2_flag = 0;
    }
    if (key3_flag == 1) {
        Speed_Ref -= 5.0f;
        key3_flag = 0;
    }
}

