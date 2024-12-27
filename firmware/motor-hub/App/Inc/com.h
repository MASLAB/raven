#ifndef INC_COM_H_
#define INC_COM_H_

#include "main.h"

struct Com_Handle {
	// configuration
    UART_HandleTypeDef* huart;
    DMA_HandleTypeDef* hdma;
    CRC_HandleTypeDef* hcrc;

    void (*receive)(uint8_t*,uint8_t);

    uint8_t sByte;
    uint8_t maxRetries;

	// internal
    uint8_t tx_pid;
    uint8_t rx_pid;

    uint8_t* rx_buf;
    uint8_t* data_buf;
    uint8_t* tx_buf;
};

// 1: start
// 2: length (5bit), ID (2bit), ack (1bit)
// : data bytes (0-32 bytes)
// : crc-8

void Com_Init(struct Com_Handle* handle);

void Com_Receive(struct Com_Handle* handle, uint16_t size);
void Com_Send(struct Com_Handle* handle, uint8_t* data, uint8_t len, uint8_t awk);


#endif