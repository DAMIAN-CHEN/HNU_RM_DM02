//
// Created by DAMIAN_CHEN on 25-3-31.
//

#ifndef UAV_TASK_H
#define UAV_TASK_H


#include "cmsis_os.h"
#include "imu_task.h"
#include "pid.h"


typedef struct
{
    pid_obj_t *pid_control;
    pid_config_t *pid_cfg;
}imu_pid_t;

typedef struct
{
    float x;
    float y;
    float z;
}imu_ref_acc_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
}imu_ref_angle_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
}imu_out_rate_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float accl_x;
    float accl_y;
    float accl_z;
}imu_mail_data_t;
void UAV_Control_loop(void);


#endif //UAV_TASK_H
