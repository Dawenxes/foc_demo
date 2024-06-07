//
// Created by begre on 2024-06-04.
//

#ifndef FOC_DEMO_LOWS_TASK_H
#define FOC_DEMO_LOWS_TASK_H

#include "main.h"

extern uint16_t hz_100_cnt;
extern uint8_t motor_start_stop;
extern enum FOC_STATE {
    MOTOR_STOP, MOTOR_GET_OFFSET, MOTOR_RUN
};
extern enum FOC_STATE foc_state;

void low_control_task(void);

#endif //FOC_DEMO_LOWS_TASK_H
