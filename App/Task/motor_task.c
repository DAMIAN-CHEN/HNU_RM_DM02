/*
* Change Logs:
* Date            Author          Notes
* 2024-10-26      陈思翰ChenSihan     1.0.0version
*
*/

#include "motor_task.h"

int16_t chassis_motor_current[4]={0};
int16_t  gimbal_motor_current[4]={0};
int16_t   shoot_motor_current[4]={0};

void Motor_Task_Entry(void const * argument)
{
    /* USER CODE BEGIN Motor_Task_Entry */
    can_bsp_init();
    /* Infinite loop */
    for(;;)
    {
        CAN_cmd_chassis(chassis_motor_current[0],chassis_motor_current[1],chassis_motor_current[2],chassis_motor_current[3]);
        CAN_cmd_gimbal(gimbal_motor_current[0],gimbal_motor_current[1],gimbal_motor_current[2],gimbal_motor_current[3]);

        osDelay(1);
    }
    /* USER CODE END Motor_Task_Entry */
}