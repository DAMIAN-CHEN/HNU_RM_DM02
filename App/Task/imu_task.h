/*
* Change Logs:
* Date            Author          Notes
* 2024-9-11      陈思翰ChenSihan     1.0.0version
*
*/
#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "cmsis_os.h"
#include "main.h"
#include "bmi088driver.h"
#include "MahonyAHRS.h"
#include "math.h"
#include "tim.h"
#include "ws2812.h"
#include "usbd_cdc_if.h"
#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2


#endif
