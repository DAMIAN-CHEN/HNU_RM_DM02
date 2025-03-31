//
// Created by DAMIAN_CHEN on 25-3-31.
//

#include "uav_task.h"

imu_pid_t r_acc_pid,r_angle_pid;
imu_pid_t p_acc_pid,p_angle_pid;
imu_pid_t y_acc_pid,y_angle_pid;

imu_ref_acc_t      imu_ref_acc[3];
imu_ref_angle_t    imu_ref_angle[3];
imu_out_rate_t     pid_out_rate;
imu_mail_data_t    imu_mail_data;
float motor_speed[4];
void uav_pid_init(void)
{
    r_acc_pid.pid_cfg=init_pid_cfg(20, 0.1f, 0.001f, 2000, 16000,0,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    r_acc_pid.pid_control= pid_register(r_acc_pid.pid_cfg);

    r_angle_pid.pid_cfg=init_pid_cfg(20, 0.1f, 0.001f, 2000, 16000,0,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    r_angle_pid.pid_control= pid_register(r_acc_pid.pid_cfg);

    p_acc_pid.pid_cfg=init_pid_cfg(20, 0.1f, 0.001f, 2000, 16000,0,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    p_acc_pid.pid_control= pid_register(r_acc_pid.pid_cfg);

    p_angle_pid.pid_cfg=init_pid_cfg(20, 0.1f, 0.001f, 2000, 16000,0,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    p_angle_pid.pid_control= pid_register(r_acc_pid.pid_cfg);

    y_acc_pid.pid_cfg=init_pid_cfg(20, 0.1f, 0.001f, 2000, 16000,0,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    y_acc_pid.pid_control= pid_register(r_acc_pid.pid_cfg);

    y_angle_pid.pid_cfg=init_pid_cfg(20, 0.1f, 0.001f, 2000, 16000,0,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    y_angle_pid.pid_control= pid_register(r_acc_pid.pid_cfg);
}
void UAV_Control_loop(void)
{

    imu_ref_acc->x=pid_calculate(r_angle_pid.pid_control,imu_mail_data.roll,imu_ref_angle->roll);
    pid_out_rate.roll=pid_calculate(r_acc_pid.pid_control,imu_mail_data.accl_x,imu_ref_acc->x);

    imu_ref_acc->y=pid_calculate(r_angle_pid.pid_control,imu_mail_data.pitch,imu_ref_angle->pitch);
    pid_out_rate.pitch=pid_calculate(r_acc_pid.pid_control,imu_mail_data.accl_y,imu_ref_acc->y);

    imu_ref_acc->z=pid_calculate(r_angle_pid.pid_control,imu_mail_data.yaw,imu_ref_angle->yaw);
    pid_out_rate.yaw=pid_calculate(r_acc_pid.pid_control,imu_mail_data.accl_z,imu_ref_acc->z);

}
/*
 *
 *         Printed by DAMIAN_CHEN
                  ◄───roll───►
 *                     X
                  M1   |   M2
                   \   |   /
                    \  |  /
                     \ | /                      ▲
      ▲ yaw ▼    -----OZO---------- Y         pitch
                     / | \                      ▼
                    /  |  \
                   /   |   \
                  M4        M3
                                               */

void UAV_Speed_Distribute(void)
{
    motor_speed[0]=+pid_out_rate.roll-pid_out_rate.pitch-pid_out_rate.yaw;
    motor_speed[1]=-pid_out_rate.roll-pid_out_rate.pitch+pid_out_rate.yaw;
    motor_speed[2]=-pid_out_rate.roll+pid_out_rate.pitch-pid_out_rate.yaw;
    motor_speed[3]=+pid_out_rate.roll+pid_out_rate.pitch+pid_out_rate.yaw;

}

/**
* @brief Function implementing the Cmd_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Cmd_Task_Entry */
void Cmd_Task_Entry(void const * argument)
{
    /* USER CODE BEGIN Cmd_Task_Entry */
    uav_pid_init();

    /* Infinite loop */
    for(;;)
    {
        UAV_Control_loop();

        osDelay(1);
    }
    /* USER CODE END Cmd_Task_Entry */
}