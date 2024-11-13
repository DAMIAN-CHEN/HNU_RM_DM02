/*
* Change Logs:
* Date            Author          Notes
* 2024-11-13      陈思翰ChenSihan     1.0.0version
*
*/

#include "chassis_task.h"

/*电机要传送的电流*/
extern int16_t chassis_motor_current[4];

/*!构建一个电机pid实例
 *
 * pid_obj_t:pid控制器实例
 *
 * pid_config_t:pid配置实例
 *
 * motor_measure_t:电机测量反馈数据
 * */
 typedef struct
 {
    pid_obj_t *pid_control;
    pid_config_t *pid_cfg;
    motor_measure_t measure;
        }motor_pid_t;


/*根据实例创建电机的控制*/
motor_pid_t chassis_motor_1_speed;




void chassis_motor_init()
{
    chassis_motor_1_speed.pid_cfg=init_pid_cfg(20, 0.1f, 0.001f, 2000, 16000,0,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    chassis_motor_1_speed.pid_control= pid_register(chassis_motor_1_speed.pid_cfg);
}



float chassis_motor_1_clc(float pid_measure,float ref)
{
    static float set;
    set=pid_calculate(chassis_motor_1_speed.pid_control,pid_measure,ref);
    return set;
}



void chassis_motor_get_measure()
{
    chassis_motor_1_speed.measure= *get_chassis_motor_measure_point(1);
}
/* USER CODE END Header_Chassis_Task_Entry */

float test_ref=500;
void Chassis_Task_Entry(void const * argument)
{


    /* USER CODE BEGIN Chassis_Task_Entry */
    chassis_motor_init();


    /* Infinite loop */
    for(;;)
    {

        chassis_motor_get_measure();

        chassis_motor_current[1]=(int16_t)chassis_motor_1_clc( chassis_motor_1_speed.measure.speed_rpm,test_ref);

        osDelay(1);
    }
    /* USER CODE END Chassis_Task_Entry */
}