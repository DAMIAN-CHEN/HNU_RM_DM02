/*
* Change Logs:
* Date            Author          Notes
* 2024-9-11      陈思翰ChenSihan     1.0.0version
*
*/
#include "trans_task.h"


extern float gyro[3];
extern float acc[3];
extern float imuQuat[4];
extern float imuAngle[3];

static int32_t rpy_data;
static uint8_t rpy_buffer[25];

void Trans_Task_Entry(void const * argument)
{
    /* USER CODE BEGIN Motor_Task_Entry */
    /* Infinite loop */
    for(;;)
    {
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
        /*虚拟串口发送数据*/
        CDC_Transmit_HS(rpy_buffer, sizeof(rpy_buffer));
        /*LED灯*/
        led_blinky_a();

        osDelay(1);
    }
    /* USER CODE END Motor_Task_Entry */
}