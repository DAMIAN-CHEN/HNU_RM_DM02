/*
* Change Logs:
* Date            Author          Notes
* 2024-9-11      陈思翰ChenSihan     1.0.0version
*
*/
#include "imu_task.h"

#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

#define DATA_LENS 24

float gyro[3] = {0.0f};
float acc[3] = {0.0f};
static float temp = 0.0f;

float imuQuat[4] = {0.0f};
float imuAngle[3] = {0.0f};

float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;

static int32_t rpy_data;
static uint8_t rpy_buffer[25];

typedef  struct
{
    uint8_t HEAD;  				          /*! 帧头 */
    //uint8_t D_ADDR;                 /*! 目标地址 */
    //uint8_t ID;                     /*! 功能码 */
    uint8_t LEN;                    /*! 数据长度 */
    uint8_t DATA[DATA_LENS];         /*! 数据内容 */
}__attribute__((packed)) DataTypeDef;

static DataTypeDef *tx_data; 

/*void add_data_to_frame(uint8_t *data_buf ,uint8_t lens)
{
	tx_data->HEAD=0xFD;
	tx_data->LEN=lens;
	memcpy(tx_data->DATA, data_buf, sizeof(tx_data->DATA));
}*/
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
        
        err_ll = err_l;
        err_l = err;
        err = DES_TEMP - temp;
        out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
        if (out > MAX_OUT) out = MAX_OUT;
        if (out < 0) out = 0.f;
        htim3.Instance->CCR4 = (uint16_t)out;
			
			  /*帧头*/
			  rpy_buffer[0]=0xFC;
			  /*加速度1*/
        rpy_data=(int32_t)(acc[0]*1000);
			  rpy_buffer[1]=rpy_data;
			  rpy_buffer[2]=rpy_data>>8;
			  rpy_buffer[3]=rpy_data>>16;
			  rpy_buffer[4]=rpy_data>>24;
			  /*加速度2*/
			  rpy_data=(int32_t)(acc[1]*1000);
			  rpy_buffer[5]=rpy_data;
			  rpy_buffer[6]=rpy_data>>8;
			  rpy_buffer[7]=rpy_data>>16;
			  rpy_buffer[8]=rpy_data>>24;
				/*加速度3*/
				rpy_data=(int32_t)(acc[2]*1000);
			  rpy_buffer[9]=rpy_data;
			  rpy_buffer[10]=rpy_data>>8;
			  rpy_buffer[11]=rpy_data>>16;
			  rpy_buffer[12]=rpy_data>>24;
				/*欧拉角1*/
				rpy_data=(int32_t)(imuAngle[0]*1000);
			  rpy_buffer[13]=rpy_data;
			  rpy_buffer[14]=rpy_data>>8;
			  rpy_buffer[15]=rpy_data>>16;
			  rpy_buffer[16]=rpy_data>>24;
				/*欧拉角2*/
				rpy_data=(int32_t)(imuAngle[1]*1000);
			  rpy_buffer[17]=rpy_data;
			  rpy_buffer[18]=rpy_data>>8;
			  rpy_buffer[19]=rpy_data>>16;
			  rpy_buffer[20]=rpy_data>>24;
				/*欧拉角3*/
				rpy_data=(int32_t)(imuAngle[2]*1000);
			  rpy_buffer[21]=rpy_data;
			  rpy_buffer[22]=rpy_data>>8;
			  rpy_buffer[23]=rpy_data>>16;
			  rpy_buffer[24]=rpy_data>>24;
			
			CDC_Transmit_HS(rpy_buffer, sizeof(rpy_buffer));
        led_blinky_a();
        osDelay(1);
    }
    /* USER CODE END ImuTask_Entry */
}



