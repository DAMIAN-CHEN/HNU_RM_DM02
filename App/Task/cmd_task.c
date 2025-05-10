//
// Created by MR on 24-11-10.
//

#include "cmd_task.h"
uav_status_t uav_now_status;
uav_status_t uav_last_status;
void Cmd_Task_Entry(void const * argument)
{
    /* USER CODE BEGIN Cmd_Task_Entry */


    /* Infinite loop */
    for(;;)
    {
        if (rc_data.swB==IBUS_SW_DN) {
            uav_now_status.lock_status=UAV_UNLOCK;
        }
        else {
            uav_now_status.lock_status=UAV_LOCK;
        }

        uav_last_status=uav_now_status;
        osDelay(1);
    }
    /* USER CODE END Cmd_Task_Entry */
}