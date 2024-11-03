/*
* Change Logs:
* Date            Author          Notes
* 2024-10-26      陈思翰ChenSihan     1.0.0version
*
*/

#include "motor_task.h"
void Motor_Task_Entry(void const * argument)
{
    /* USER CODE BEGIN Motor_Task_Entry */
    can_bsp_init();
    /* Infinite loop */
    for(;;)
    {
        CAN_cmd_chassis(6000,6000,6000,6000);
        CAN_cmd_gimbal(6000,6000,6000,6000);



        osDelay(1);
    }
    /* USER CODE END Motor_Task_Entry */
}