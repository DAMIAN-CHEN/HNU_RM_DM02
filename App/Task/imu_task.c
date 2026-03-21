/**
 * @file    imu_task.c
 * @brief   IMU 采集、姿态解算与上位机数据上传任务实现.
 *
 * Change Logs:
 * Date         Author          Notes
 * 2024-09-11   陈思翰 ChenSihan  初始版本 1.0.0
 */
#include "imu_task.h"

#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

#define DATA_LENS 24

float gyro[3] = {0.0f};
float acc[3] = {0.0f};
static float temp = 0.0f;

float imuQuat[4] = {0.0f};
float imuAngle[3] = {0.0f};

float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;

/**
 * @name IMU 上行数据帧定义
 * @brief 帧格式: SOF(1) + TYPE(1) + LEN(1) + PAYLOAD(LEN) + CRC16(2).
 * @{
 */
#define IMU_FRAME_SOF        0xFC
#define IMU_FRAME_TYPE_IMU   0x01
#define IMU_FRAME_PAYLOAD_LEN 24u   /**< 负载长度: 6 * int32_t = 24 字节 */
#define IMU_FRAME_FIXED_LEN   (1u + 1u + 1u + IMU_FRAME_PAYLOAD_LEN + 2u) /**< 完整帧长度: 29 字节 */
/** @} */

static int32_t rpy_data;
static uint8_t rpy_buffer[32];

/**
 * @brief 通用数据帧结构体（示例，当前未实际使用）.
 */
typedef struct
{
    uint8_t HEAD;                 /**< 帧头 */
    /* uint8_t D_ADDR; */        /**< 目标地址（预留） */
    /* uint8_t ID;     */        /**< 功能码（预留） */
    uint8_t LEN;                  /**< 数据长度 */
    uint8_t DATA[DATA_LENS];      /**< 数据区 */
} __attribute__((packed)) DataTypeDef;

static DataTypeDef *tx_data; 

/*void add_data_to_frame(uint8_t *data_buf ,uint8_t lens)
{
	tx_data->HEAD=0xFD;
	tx_data->LEN=lens;
	memcpy(tx_data->DATA, data_buf, sizeof(tx_data->DATA));
}*/
void AHRS_init(float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(float quat[4], float gyro[3], float accel[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

/**
 * @brief  计算 CRC16-IBM 校验值.
 * @param  data 指向待校验数据缓冲区.
 * @param  len  数据长度（字节数）.
 * @return 计算得到的 CRC16 校验值（多项式 0xA001，初值 0xFFFF）.
 */
static uint16_t IMU_CRC16_Calc(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t  j;

    for (i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief  构造并发送 IMU 数据帧.
 *
 * 帧类型 TYPE = 0x01，PAYLOAD 内容为 3 轴加速度 + 3 轴姿态角，
 * 均以 int32 形式发送，数值放大 1000 倍（单位 1e-3）.
 */
static void IMU_SendFrame(void)
{
    uint8_t *buf = rpy_buffer;
    uint16_t crc;
    uint8_t idx = 0;

    /* 写入帧头 SOF */
    buf[idx++] = IMU_FRAME_SOF;
    /* 写入帧类型 TYPE */
    buf[idx++] = IMU_FRAME_TYPE_IMU;
    /* 写入负载长度 LEN */
    buf[idx++] = IMU_FRAME_PAYLOAD_LEN;

    /* PAYLOAD 起始: 依次写入 acc[0..2], imuAngle[0..2] (单位: 1e-3) */
    /* 加速度 X */
    rpy_data = (int32_t)(acc[0] * 1000.0f);
    buf[idx++] = (uint8_t)(rpy_data);
    buf[idx++] = (uint8_t)(rpy_data >> 8);
    buf[idx++] = (uint8_t)(rpy_data >> 16);
    buf[idx++] = (uint8_t)(rpy_data >> 24);
    /* 加速度 Y */
    rpy_data = (int32_t)(acc[1] * 1000.0f);
    buf[idx++] = (uint8_t)(rpy_data);
    buf[idx++] = (uint8_t)(rpy_data >> 8);
    buf[idx++] = (uint8_t)(rpy_data >> 16);
    buf[idx++] = (uint8_t)(rpy_data >> 24);
    /* 加速度 Z */
    rpy_data = (int32_t)(acc[2] * 1000.0f);
    buf[idx++] = (uint8_t)(rpy_data);
    buf[idx++] = (uint8_t)(rpy_data >> 8);
    buf[idx++] = (uint8_t)(rpy_data >> 16);
    buf[idx++] = (uint8_t)(rpy_data >> 24);
    /* 姿态 yaw */
    rpy_data = (int32_t)(imuAngle[0] * 1000.0f);
    buf[idx++] = (uint8_t)(rpy_data);
    buf[idx++] = (uint8_t)(rpy_data >> 8);
    buf[idx++] = (uint8_t)(rpy_data >> 16);
    buf[idx++] = (uint8_t)(rpy_data >> 24);
    /* 姿态 pitch */
    rpy_data = (int32_t)(imuAngle[1] * 1000.0f);
    buf[idx++] = (uint8_t)(rpy_data);
    buf[idx++] = (uint8_t)(rpy_data >> 8);
    buf[idx++] = (uint8_t)(rpy_data >> 16);
    buf[idx++] = (uint8_t)(rpy_data >> 24);
    /* 姿态 roll */
    rpy_data = (int32_t)(imuAngle[2] * 1000.0f);
    buf[idx++] = (uint8_t)(rpy_data);
    buf[idx++] = (uint8_t)(rpy_data >> 8);
    buf[idx++] = (uint8_t)(rpy_data >> 16);
    buf[idx++] = (uint8_t)(rpy_data >> 24);

    /* 计算 CRC16: 对 TYPE+LEN+PAYLOAD 共 1 + 1 + 24 = 26 字节做校验 */
    crc = IMU_CRC16_Calc(&buf[1], (uint16_t)(1u + 1u + IMU_FRAME_PAYLOAD_LEN));
    /* 低字节在前，高字节在后 */
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    /* 发送完整数据帧 */
    (void)CDC_Transmit_HS(buf, idx);
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
void Imu_Task_Entry(void const * argument)
{
    /* USER CODE BEGIN ImuTask_Entry */
    osDelay(10);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init())
    {
        osDelay(100);
    }
    
    AHRS_init(imuQuat);
    /* Infinite loop */
    for(;;)
    {
        BMI088_read(gyro, acc, &temp);
        
        AHRS_update(imuQuat, gyro, acc);
        GetAngle(imuQuat, imuAngle + INS_YAW_ADDRESS_OFFSET, imuAngle + INS_PITCH_ADDRESS_OFFSET, imuAngle + INS_ROLL_ADDRESS_OFFSET);
        
        err_ll = err_l;
        err_l = err;
        err = DES_TEMP - temp;
        out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
        if (out > MAX_OUT) out = MAX_OUT;
        if (out < 0) out = 0.f;
        htim3.Instance->CCR4 = (uint16_t)out;
        
        /* 通过统一协议帧发送 IMU 数据 */
        IMU_SendFrame();
        osDelay(1);
    }
    /* USER CODE END ImuTask_Entry */
}



