#include "com.h"
#include "string.h"
#include "stdlib.h"

void Com_Init(struct Com_Handle* handle) {
    handle->rx_pid = 0;
    handle->tx_pid = 1;

    handle->rx_buf = malloc(35);
    handle->data_buf = malloc(32);

    HAL_UARTEx_ReceiveToIdle_DMA(handle->huart, handle->rx_buf, 35);
    __HAL_DMA_DISABLE_IT(handle->hdma, DMA_IT_HT);
}

void Com_Receive(struct Com_Handle* handle, uint16_t size) {
    const uint8_t len = handle->rx_buf[1]>>3;
    if (size == len+3) {
        if ((HAL_CRC_Calculate(handle->hcrc, (uint32_t*)handle->rx_buf, size)&0xFF) == 0) {//include CRC byte
            const uint8_t pid = (handle->rx_buf[1]>>1)&3;
            if (pid != handle->rx_pid) {
                handle->receive(handle->data_buf, len);
                handle->rx_pid++;
                if (handle->rx_buf[1]&1) {
                    //TODO send back ok message
                }
            }
        }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(handle->huart, handle->rx_buf, 35);
    __HAL_DMA_DISABLE_IT(handle->hdma, DMA_IT_HT);
}

void Com_Send(struct Com_Handle* handle, uint8_t* data, uint8_t len, uint8_t awk) {
    handle->tx_buf[1] = (len << 3) | ((handle->tx_pid & 3) << 1) | (awk & 1);
    memcpy(handle->tx_buf+2, data, len);
    const uint8_t crc = HAL_CRC_Calculate(handle->hcrc, (uint32_t*)handle->tx_buf, len+2);
    handle->tx_buf[len+2] = crc;
    if (!awk) {
        handle->tx_pid++;
    }else {
        //TODO later make sure ok to pid++ or resend
    }
}