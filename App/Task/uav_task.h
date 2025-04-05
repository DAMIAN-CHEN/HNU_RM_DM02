//
// Created by DAMIAN_CHEN on 25-3-31.
//

#ifndef UAV_TASK_H
#define UAV_TASK_H


#include "cmsis_os.h"
#include "imu_task.h"
#include "pid.h"
#include "ibus_task.h"
#include "cmd_task.h"

extern float motor_speed_pwm[4];
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

/*-------------------------------------------------------------*/
#define YAW_ANGLE_KP                     1
#define YAW_ANGLE_KI                     0
#define YAW_ANGLE_KD                     0
#define YAW_ANGLE_MAXIOUT                0
#define YAW_ANGLE_MAXOUT                 30
#define YAW_ANGLE_DEADBAND               0.3f

#define YAW_ACCL_KP                      140
#define YAW_ACCL_KI                      0
#define YAW_ACCL_KD                      0.01f
#define YAW_ACCL_MAXIOUT                 0
#define YAW_ACCL_MAXOUT                  8000
#define YAW_ACCL_DEADBAND                0.5f


#define PITCH_ANGLE_KP                   1
#define PITCH_ANGLE_KI                   0
#define PITCH_ANGLE_KD                   0
#define PITCH_ANGLE_MAXIOUT              0
#define PITCH_ANGLE_MAXOUT               30
#define PITCH_ANGLE_DEADBAND             0

#define PITCH_ACCL_KP                    140
#define PITCH_ACCL_KI                    0
#define PITCH_ACCL_KD                    0.01f
#define PITCH_ACCL_MAXIOUT               0
#define PITCH_ACCL_MAXOUT                8000
#define PITCH_ACCL_DEADBAND              0.5f


#define ROLL_ANGLE_KP                    1
#define ROLL_ANGLE_KI                    0
#define ROLL_ANGLE_KD                    0
#define ROLL_ANGLE_MAXIOUT               0
#define ROLL_ANGLE_MAXOUT                30
#define ROLL_ANGLE_DEADBAND              0

#define ROLL_ACCL_KP                     140
#define ROLL_ACCL_KI                     0
#define ROLL_ACCL_KD                     0.01f
#define ROLL_ACCL_MAXIOUT                0
#define ROLL_ACCL_MAXOUT                 8000
#define ROLL_ACCL_DEADBAND               0.5f

/*-------------------------------------------------------------*/
static void UAV_Control_loop(void);
static void UAV_Speed_Distribute(void);
extern imu_mail_data_t imu_mail_data;

#endif //UAV_TASK_H
