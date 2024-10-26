/*
* Change Logs:
* Date            Author          Notes
* 2024-9-11      陈思翰ChenSihan     1.0.0version 
* 
*/
#include"boat_task.h"

/*从树莓派接收数据*/
extern uint8_t Rpi_buffer[5];
static uint8_t Rpi_data[5];

 /* PWM输出通道 */
static int ccr_num_pwm1=24000-1;
static int ccr_num_pwm2=24000-1;
static int ccr_num_pwm3=24000-1;
static int ccr_num_pwm4=24000-1;

 /* 电机输出量 */
static float servo_number=0.075f;
static float motor_number=0.075f;

/*PWM波输出函数*/
void BOAT_PWM_OUT(int pwm_channel,float pwm_period)
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
 void Boat_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN BoatTask_Entry */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	
  /* Infinite loop */
  for(;;)
  {
		/*转存数据*/
		memcpy(Rpi_data,Rpi_buffer,5);
		/*拼接数据*/
		if(Rpi_data[0]==0xFF)
    {
			servo_number=(float)((uint16_t)(Rpi_data[1]<<8|Rpi_data[2]))/10000;
			motor_number=(float)((uint16_t)(Rpi_data[3]<<8|Rpi_data[4]))/10000;
    }


		/****************************
		舵机角度限位归中值0.075
		左上限0.067
		右上限0.081
		*****************************/
		if(servo_number<=0.065f)servo_number=0.065f;
		if(servo_number>=0.085f)servo_number=0.085f;
		BOAT_PWM_OUT(1,servo_number);//舵机
		
		/****************************
		无刷电机电调归中值0.075 
		油门虚位起转点0.072~0.078
		后上限0.070
		前上限0.080
		****************************/
		if(motor_number<=0.065f)motor_number=0.065f;
		if(motor_number>=0.085f)motor_number=0.085f;
		BOAT_PWM_OUT(2,motor_number);//无刷电机

  osDelay(1);/*！线程切换边缘勿动*/	
  }
	
  /* USER CODE END BoatTask_Entry */
}