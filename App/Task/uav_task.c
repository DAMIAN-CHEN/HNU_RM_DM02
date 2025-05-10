//
// Created by DAMIAN_CHEN on 25-3-31.
//

#include "uav_task.h"



imu_pid_t roll_acc_pid,roll_angle_pid;
imu_pid_t pitch_acc_pid,pitch_angle_pid;
imu_pid_t yaw_acc_pid,yaw_angle_pid;

imu_ref_acc_t      imu_ref_acc;
imu_ref_angle_t    imu_ref_angle;
imu_out_rate_t     pid_out_rate;
imu_mail_data_t    imu_mail_data;
static imu_mail_data_t    imu_mail_data_rad;

float motor_speed[4];
float motor_speed_pwm[4];
static float throttle;
static float uav_task_period_us;
static void uav_pid_init(void)
{
    roll_acc_pid.pid_cfg=init_pid_cfg(ROLL_ACCL_KP, ROLL_ACCL_KI, ROLL_ACCL_KD, ROLL_ACCL_MAXIOUT,\
        ROLL_ACCL_MAXOUT,ROLL_ACCL_DEADBAND,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    roll_acc_pid.pid_control= pid_register(roll_acc_pid.pid_cfg);

    roll_angle_pid.pid_cfg=init_pid_cfg(ROLL_ANGLE_KP, ROLL_ANGLE_KI, ROLL_ANGLE_KD, ROLL_ANGLE_MAXIOUT, \
        ROLL_ANGLE_MAXOUT,ROLL_ANGLE_DEADBAND,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    roll_angle_pid.pid_control= pid_register(roll_angle_pid.pid_cfg);

    pitch_acc_pid.pid_cfg=init_pid_cfg(PITCH_ACCL_KP, PITCH_ACCL_KI, PITCH_ACCL_KD, PITCH_ACCL_MAXIOUT, \
        PITCH_ACCL_MAXOUT,PITCH_ACCL_DEADBAND,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    pitch_acc_pid.pid_control= pid_register(pitch_acc_pid.pid_cfg);

    pitch_angle_pid.pid_cfg=init_pid_cfg(PITCH_ANGLE_KP, PITCH_ANGLE_KI, PITCH_ANGLE_KD, PITCH_ANGLE_MAXIOUT, \
        PITCH_ANGLE_MAXOUT,PITCH_ANGLE_DEADBAND,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    pitch_angle_pid.pid_control= pid_register(pitch_angle_pid.pid_cfg);

    yaw_acc_pid.pid_cfg=init_pid_cfg(YAW_ACCL_KP, YAW_ACCL_KI, YAW_ACCL_KD, YAW_ACCL_MAXIOUT, \
        YAW_ACCL_MAXOUT,YAW_ACCL_DEADBAND,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    yaw_acc_pid.pid_control= pid_register(yaw_acc_pid.pid_cfg);

    yaw_angle_pid.pid_cfg=init_pid_cfg(YAW_ANGLE_KP, YAW_ANGLE_KI, YAW_ANGLE_KD, YAW_ANGLE_MAXIOUT, \
        YAW_ANGLE_MAXOUT,YAW_ANGLE_DEADBAND,PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement);
    yaw_angle_pid.pid_control= pid_register(yaw_angle_pid.pid_cfg);
}
void UAV_Control_loop(void)
{
    UAV_IMU_Conversion_of_Units();//IMU单位换算成角度

    UAV_Cmd_Deal();//命令处理

    UAV_Pid_Calculate();//串级PID计算

    UAV_Speed_Distribute();//动力分配
}
/*TODO：
 *
 *         Printed by DAMIAN_CHEN
                  ◄───roll───►
 *                <-    Y  ->
                  M0   |   M1
                  ->       <-
                   \   |   /
                    \  |  /
                     \ | /                      ▲
      ▲ yaw ▼   X-----OZO----------          pitch
                     / | \                      ▼
                    /  |  \
                   /   |   \
                   ->       <-
                  M3        M2
                  <-        ->
         该项目算法下，DM-02IMU参数正负整定
         板子摆放-------->水平正放在桌面上XT30头对应正前方
         ACCL        x左正右负                Y前负后正          Z下正上负
         ANGLE     roll 左下倾负右下倾正 pitch前下倾负后下倾正   yaw左转负右转正

                                               */

static void UAV_Speed_Distribute(void)
{
    if (rc_data.ch3<=1500) {
        throttle=(-((float)rc_data.ch3-1500))*10; //暂定最大油门开度5000
    }else {
        throttle=0;
    }
    motor_speed[0]=throttle+pid_out_rate.roll+pid_out_rate.pitch-pid_out_rate.yaw;
    motor_speed[1]=throttle-pid_out_rate.roll+pid_out_rate.pitch+pid_out_rate.yaw;
    motor_speed[2]=throttle-pid_out_rate.roll-pid_out_rate.pitch-pid_out_rate.yaw;
    motor_speed[3]=throttle+pid_out_rate.roll-pid_out_rate.pitch+pid_out_rate.yaw;

    for (int i = 0; i < 4; ++i)
    {
        if (motor_speed[i]>=5000)  motor_speed[i]=5000;
        else if (motor_speed[i]<=0) motor_speed[i]=0;
        motor_speed_pwm[i]=motor_speed[i]/5000*0.05f;
    }
}

static void UAV_Pid_Calculate(void)
{
    imu_ref_acc.x=pid_calculate(roll_angle_pid.pid_control,imu_mail_data_rad.roll,imu_ref_angle.roll);
    pid_out_rate.roll=pid_calculate(roll_acc_pid.pid_control,imu_mail_data_rad.accl_x,imu_ref_acc.x);

    imu_ref_acc.y=pid_calculate(pitch_angle_pid.pid_control,imu_mail_data_rad.pitch,imu_ref_angle.pitch);
    pid_out_rate.pitch=pid_calculate(pitch_acc_pid.pid_control,imu_mail_data_rad.accl_y,imu_ref_acc.y);

    imu_ref_acc.z=pid_calculate(yaw_angle_pid.pid_control,imu_mail_data_rad.yaw,imu_ref_angle.yaw);
    pid_out_rate.yaw=pid_calculate(yaw_acc_pid.pid_control,imu_mail_data_rad.accl_z,imu_ref_acc.z);
}

static void UAV_IMU_Conversion_of_Units(void)
{
    //单位:rad 度
    imu_mail_data_rad.yaw=imu_mail_data.yaw*57.296f;
    imu_mail_data_rad.pitch=imu_mail_data.pitch*57.296f;
    imu_mail_data_rad.roll=imu_mail_data.roll*57.296f;
    //单位:米每秒的平方 m/s^2
    imu_mail_data_rad.accl_x=imu_mail_data.accl_x;
    imu_mail_data_rad.accl_y=imu_mail_data.accl_y;
    imu_mail_data_rad.accl_z=imu_mail_data.accl_z;
}
static void UAV_Cmd_Deal(void)
{
    /*富斯i6遥控器的中点值为1500，最低点1000，最高点2000*/
    if (uav_now_status.lock_status==UAV_UNLOCK)
    {
        imu_ref_angle.yaw-=((float)rc_data.ch4-1500)*0.0001f; //500*0.0001推到底期望增加每次增加0.0025（yaw轴要保持，因此是累加角度）
        imu_ref_angle.pitch=((float)rc_data.ch2-1500)*0.06f; //500*0.06推到底期望增加最高增加到30
        imu_ref_angle.roll=-((float)rc_data.ch1-1500)*0.06f;  //500*0.06推到底期望增加最高增加到30
    }
    else {
        imu_ref_angle.roll=0;
        imu_ref_angle.pitch=0;
        imu_ref_angle.yaw=imu_mail_data_rad.yaw;  //
    }
}

/**
* @brief Function implementing the Cmd_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Cmd_Task_Entry */
void UAV_Task_Entry(void const * argument)
{
    /* USER CODE BEGIN Cmd_Task_Entry */
    uav_pid_init();
    uint64_t start,end;
    /* Infinite loop */
    for(;;)
    {
        start=dwt_get_time_us();

        UAV_Control_loop();

        end = dwt_get_time_us()-start;
        uav_task_period_us=end;

        osDelay(1);
    }
    /* USER CODE END Cmd_Task_Entry */
}