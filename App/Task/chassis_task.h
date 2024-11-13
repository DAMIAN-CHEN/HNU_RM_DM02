/*
* Change Logs:
* Date            Author          Notes
* 2024-11-13      陈思翰ChenSihan     1.0.0version
*
*/

#ifndef HNU_RM_DM02_CHASSIS_TASK_H
#define HNU_RM_DM02_CHASSIS_TASK_H
#include "cmsis_os.h"
#include "pid.h"
#include "bsp_can.h"


void chassis_motor_get_measure();
void chassis_motor_init();
float chassis_motor_1_clc(float pid_measure,float ref);
#endif //HNU_RM_DM02_CHASSIS_TASK_H
