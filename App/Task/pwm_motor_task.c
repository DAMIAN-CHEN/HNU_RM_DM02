/*
* Change Logs:
* Date            Author          Notes
* 2024-9-11      陈思翰ChenSihan     1.0.0version 
* 
*/
#include"pwm_motor_task.h"

/*从树莓派接收数据*/
extern uint8_t Rpi_buffer[5];
static uint8_t Rpi_data[5];


//#define THROTTLE_SET


 /* PWM输出通道 */
static int ccr_num_pwm1=24000-1;
static int ccr_num_pwm2=24000-1;
static int ccr_num_pwm3=24000-1;
static int ccr_num_pwm4=24000-1;

static float motor_pwm_period[4]={0};
 /* 电机输出量 */

static float pwm_motor_task_period_us;

/*PWM波输出函数*/
void Pwm_Motor_Out(int pwm_channel,float pwm_period)
{
	switch (pwm_channel)
	{
	   case 1:
		 TIM1->CCR3 = (uint32_t)((float )ccr_num_pwm1*pwm_period);
		break;

		case 2:
		 TIM1->CCR1 = (uint32_t)((float )ccr_num_pwm2*pwm_period);
		break;

		case 3:
		TIM2->CCR3 = (uint32_t)((float )ccr_num_pwm3*pwm_period);
		break;

		case 4:
		 TIM2->CCR1 = (uint32_t)((float )ccr_num_pwm4*pwm_period);
		break;
	
	default:
		break;
	}

}

 /* 线程启动点*/
 void Pwm_Motor_Task_Entry(void const * argument)
{

  /* USER CODE BEGIN BoatTask_Entry */
	uav_now_status.lock_status=UAV_LOCK;
#ifdef THROTTLE_SET
	Pwm_Motor_Out(1,MOTOR_MAX_VAL);//无刷电机
	Pwm_Motor_Out(2,MOTOR_MAX_VAL);//无刷电机
	Pwm_Motor_Out(3,MOTOR_MAX_VAL);//无刷电机
	Pwm_Motor_Out(4,MOTOR_MAX_VAL);//无刷电机
	osDelay(4000);
		Pwm_Motor_Out(1,MOTOR1_MIN_VAL);//无刷电机
		Pwm_Motor_Out(2,MOTOR2_MIN_VAL);//无刷电机
		Pwm_Motor_Out(3,MOTOR3_MIN_VAL);//无刷电机
		Pwm_Motor_Out(4,MOTOR4_MIN_VAL);//无刷电机


#else

	Pwm_Motor_Out(1,MOTOR_STOP_VAL);//无刷电机
	Pwm_Motor_Out(2,MOTOR_STOP_VAL);//无刷电机
	Pwm_Motor_Out(3,MOTOR_STOP_VAL);//无刷电机
	Pwm_Motor_Out(4,MOTOR_STOP_VAL);//无刷电机

#endif

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	
  /* Infinite loop */
  for(;;)
  {
  	float start,end;
  	start=dwt_get_time_ms();

  	if (uav_now_status.lock_status==UAV_UNLOCK)
  	{
  		motor_pwm_period[0]=MOTOR1_MIN_VAL+motor_speed_pwm[0];
  		motor_pwm_period[1]=MOTOR2_MIN_VAL+motor_speed_pwm[1];
  		motor_pwm_period[2]=MOTOR3_MIN_VAL+motor_speed_pwm[2];
  		motor_pwm_period[3]=MOTOR4_MIN_VAL+motor_speed_pwm[3];

  		Pwm_Motor_Out(1,motor_pwm_period[0]);//无刷电机
  		Pwm_Motor_Out(2,motor_pwm_period[1]);//无刷电机
  		Pwm_Motor_Out(3,motor_pwm_period[2]);//无刷电机
  		Pwm_Motor_Out(4,motor_pwm_period[3]);//无刷电机

  	}

  	if (uav_now_status.lock_status==UAV_LOCK) {
	Pwm_Motor_Out(1,MOTOR_STOP_VAL);//无刷电机
	Pwm_Motor_Out(2,MOTOR_STOP_VAL);//无刷电机
	Pwm_Motor_Out(3,MOTOR_STOP_VAL);//无刷电机
	Pwm_Motor_Out(4,MOTOR_STOP_VAL);//无刷电机
}


  	end = dwt_get_time_ms()-start;
  	pwm_motor_task_period_us=end;

  osDelay(1);/*！线程切换边缘勿动*/	
  }
	
  /* USER CODE END BoatTask_Entry */
}