/*
* Change Logs:
* Date            Author              Notes
* 2024-10-26      ³ÂË¼º²ChenSihan     1.0.0version IMUÎÂ¶ÈPID
*
*/
#include "imu_task.h"

#define DES_TEMP    30.0f
#define KP          300.f
#define KI          100.f
#define KD          10.f
#define MAX_I_OUT   100.f
#define MAX_OUT     8000.f

#define DATA_LENS 24

float gyro[3] = {0.0f};
float acc[3] = {0.0f};
static float temp = 0.0f;

float imuQuat[4] = {0.0f};
float imuAngle[3] = {0.0f};


void AHRS_init(float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(float quat[4], float gyro[3], float accel[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}


static float err = 0;
static float err_l = 0;
static float err_ll = 0;
static uint16_t IMU_Temp_Pid(float set,float measure,float Kp,float Ki,float Kd,float max_i_out,float max_out)
{
    static float out = 0;
    static float P;
    static float I;
    static float D;

    err_ll = err_l;
    err_l = err;
    err = set - measure;
    P= Kp*err;
    I= Ki*(err + err_l + err_ll);
    if (I > max_i_out) I = max_i_out;
    D= Kd*(err - err_l);
    out=P+I+D;
    if (out > max_out) out = max_out;
    if (out < 0) out = 0.f;
    return (uint16_t)out;
}
/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
void Imu_Task_Entry(void const * argument)
{
    /* USER CODE BEGIN ImuTask_Entry */
    osDelay(10);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init())
    {
        osDelay(100);
    }
    
    AHRS_init(imuQuat);
    /* Infinite loop */
    for(;;)
    {
        BMI088_read(gyro, acc, &temp);
        
        AHRS_update(imuQuat, gyro, acc);
        GetAngle(imuQuat, imuAngle + INS_YAW_ADDRESS_OFFSET, imuAngle + INS_PITCH_ADDRESS_OFFSET, imuAngle + INS_ROLL_ADDRESS_OFFSET);

        htim3.Instance->CCR4 = IMU_Temp_Pid(DES_TEMP,temp,KP,KI,KD,MAX_I_OUT,MAX_OUT);

        osDelay(1);
    }
    /* USER CODE END ImuTask_Entry */
}



