#ifndef __PWM_MOTOR_TASK_H__
#define __PWM_MOTOR_TASK_H__

#include "cmsis_os.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "drv_dwt.h"
#include "cmd_task.h"
#include "uav_task.h"

#define MOTOR_STOP_VAL   0.050f
#define MOTOR1_MIN_VAL    0.050f
#define MOTOR_MID_VAL    0.075f
#define MOTOR_MAX_VAL    0.100f

#define MOTOR1_MIN_VAL    0.050f
#define MOTOR2_MIN_VAL    0.050f
#define MOTOR3_MIN_VAL    0.050f
#define MOTOR4_MIN_VAL    0.050f

#endif