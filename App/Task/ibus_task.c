//
// Created by DAMIAN_CHEN on 25-3-30.
//

#include "ibus_task.h"


/* USER CODE BEGIN Header_Ibus_Task_Entry */
/**
* @brief Function implementing the Ibus_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ibus_Task_Entry */
void Ibus_Task_Entry(void const * argument)
{
    /* USER CODE BEGIN Ibus_Task_Entry */
    ibus_init();
    /* Infinite loop */
    for(;;)
    {


        osDelay(1);
    }
    /* USER CODE END Ibus_Task_Entry */
}