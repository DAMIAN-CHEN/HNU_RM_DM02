//
// Created by DAMIAN_CHEN on 25-3-30.
//

#include "drv_ibus.h"

#include <string.h>


static uint8_t  rx_buffer[32] = {0};

static uint8_t  rx_buffer_from_dma[32] = {0};
static uint16_t channel[IBUS_USER_CHANNELS] = {0};
static uint16_t checksum_cal, checksum_ibus;

fs_ia6b_ibus_t rc_data;

void ibus_unpack()
{
    int i = 0,j = 0;
    uint16_t channel_buffer[IBUS_MAX_CHANNLES] = {0};

    if(rx_buffer[0] == IBUS_start1 && rx_buffer[1] == IBUS_start2)
    {
        checksum_cal = 0xffff - rx_buffer[0] - rx_buffer[1];

        for( i = 0; i < IBUS_MAX_CHANNLES; i++)
        {
            channel_buffer[i] = (uint16_t)(rx_buffer[i * 2 + 3] << 8 | rx_buffer[i * 2 + 2]);
            checksum_cal = checksum_cal - rx_buffer[i * 2 + 3] - rx_buffer[i * 2 + 2];
        }

        checksum_ibus = rx_buffer[31] << 8 | rx_buffer[30];

        if(checksum_cal == checksum_ibus)
        {
            for(j = 0; j < IBUS_USER_CHANNELS; j++)
            {
                channel[j] = channel_buffer[j];
            }
            memcpy(&rc_data,channel,sizeof(fs_ia6b_ibus_t));
        }
    }
}


void ibus_init(void)
{
    HAL_UART_Receive_DMA(&huart8, rx_buffer_from_dma, sizeof(rx_buffer_from_dma));  // Start the DMA transfer
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // This callback is called when the full amount of data specified is received
    //memcpy(&rc_data,channel,sizeof(fs_ia6b_ibus_t));
    memcpy(&rx_buffer,rx_buffer_from_dma,sizeof(rx_buffer));
    //HAL_UART_Receive_DMA(&huart8, rx_buffer, sizeof(rx_buffer));  // Start the DMA transfer
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_DMA(&huart8, rx_buffer_from_dma, sizeof(rx_buffer_from_dma));  // Start the DMA transfer
}