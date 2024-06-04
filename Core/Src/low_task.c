//
// Created by begre on 2024-06-04.
//
/**********************************

**********************************/
#include "low_task.h"



uint16_t hz_100_cnt = 0;
uint8_t motor_start_stop = 0;
uint8_t motor_start_stop_pre = 1;

void motor_start(void)
{

    foc_algorithm_initialize();
    Speed_Ref=20.0F;
    speed_close_loop_flag=0;
    Iq_ref=0.0f;

    hall_angle_add=0.0005f;
    hall_speed = 0.0f;
    set_bldcm_enable();
}
void motor_stop(void)
{
    set_bldcm_disable();
}


void low_control_task(void)
{
    if(get_offset_flag == 2)
    {
        if(motor_start_stop_pre!=motor_start_stop)
        {
            motor_start_stop_pre=motor_start_stop;
            if(motor_start_stop == 1)
            {
                motor_start();
            }
            else
            {
                motor_stop();
            }

        }
    }
    if(key1_flag==1)
    {
        if(motor_start_stop==0)
        {
            motor_start_stop=1;
        }
        else
        {
            motor_start_stop=0;
        }
        key1_flag=0;
    }
    if(key2_flag==1)
    {
        Speed_Ref+=5.0f;
        key2_flag=0;
    }
    if(key3_flag==1)
    {
        Speed_Ref-=5.0f;
        key3_flag=0;
    }
}

