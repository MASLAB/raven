#include "com.h"
#include "string.h"
#include "stdlib.h"

static void Com_Callback_(UART_HandleTypeDef *huart);

// Callback handling 
static Com_Reply_t*
EmptyRead(uint8_t* message, uint8_t length) {
    static Com_Reply_t none = {
        .length = 0
    };

    return &none;
}
static bool
EmptyWrite(uint8_t* message, uint8_t length) {
    return false;
}

// Read helper functions
static void Com_Read_(Com_Handle_t* handle, uint8_t *buffer, uint8_t length) {
    HAL_UART_Receive_DMA(handle->huart, buffer, length);
}
static void Com_ReadCtrl_(Com_Handle_t* handle) {
    Com_Read_(handle, &handle->ctrlByte_, 1);
}
static void Com_ReadData_(Com_Handle_t* handle) {
    Com_Read_(handle, handle->dataBuf_, handle->dataLength_);
}

void Com_Init(Com_Handle_t* handle) {
    // Initialize values
    for (uint8_t i = 0; i < NUM_HDR; i++) {
        handle->readCallbacks_[i] = &EmptyRead;
        handle->writeCallbacks_[i] = &EmptyWrite;
    }
    handle->txBuf_[0] = handle->sByte; // Initialize to start byte
    handle->rxState_ = RX_START;

    // Initialize callback and reinit UART
    handle->huart->CallbackArg = handle;
    HAL_UART_RegisterCallback(handle->huart, HAL_UART_RX_COMPLETE_CB_ID, Com_Callback_);
    HAL_UART_Init(handle->huart);

    // Start reading first byte
    Com_ReadCtrl_(handle);
}

void Com_RegisterReadCallback(
    Com_Handle_t* handle,
    Header_Type_t header_type,
    Com_Reply_t* (*callback)(uint8_t* data, uint8_t length)
) {
    handle->readCallbacks_[header_type] = callback;
}

void Com_RegisterWriteCallback(
    Com_Handle_t* handle,
    Header_Type_t header_type,
    bool (*callback)(uint8_t* data, uint8_t length)
) {
    handle->writeCallbacks_[header_type] = callback;
}

static void Com_SendData_(Com_Handle_t* handle, Header_Type_t header, void* data, uint8_t length) {
    handle->txBuf_[1] = header;
    handle->txBuf_[2] = length;
    if(length) memcpy(&handle->txBuf_[3], data, length);
    handle->txBuf_[length + 3] = HAL_CRC_Calculate(handle->hcrc, (uint32_t) handle->txBuf_, length + 3);
    while ((HAL_UART_GetState(handle->huart) & 1) != 0)
      ;
    HAL_UART_Transmit_DMA(handle->huart, handle->txBuf_, length + 4);
}

static void Com_SendAck_(Com_Handle_t* handle, bool ack) {
    Com_SendData_(handle, ack, NULL, 0);
}

static void Com_Process_Write_(Com_Handle_t* handle) {
    bool ack = handle->writeCallbacks_[handle->header_.header](handle->dataBuf_, handle->dataLength_);
    Com_SendAck_(handle, ack);
}

static void Com_Process_Read_(Com_Handle_t* handle) {
    Com_Reply_t* reply = handle->readCallbacks_[handle->header_.header](handle->dataBuf_, handle->dataLength_);
    Com_SendData_(handle, HDR_REPLY, reply->data, reply->length);
}

void Com_Callback_(UART_HandleTypeDef *huart) {
    // State machine for parsing message
    Com_Handle_t* handle = (Com_Handle_t*) huart->CallbackArg;

    switch (handle->rxState_) {
    case RX_START:
        // Get the a reader byte if received byte is a start byte
        if (handle->ctrlByte_ == handle->sByte) {
            HAL_CRC_Calculate(handle->hcrc, (uint32_t *)&handle->ctrlByte_, 1);
            handle->rxState_ = RX_HEADER;
        }
        // Else keep read the next byte as pcf byte
        Com_ReadCtrl_(handle);
        break;
    case RX_HEADER:
        // Set header
        handle->header_.value = handle->ctrlByte_;
        if(handle->header_.header > NUM_HDR) {
            // Nack if handle is not among the expected handle
            handle->rxState_ = RX_START;
            Com_SendAck_(handle, 0);
        } else {
            // Else get length
            HAL_CRC_Accumulate(handle->hcrc, (uint32_t *)&handle->ctrlByte_, 1);
            handle->rxState_ = RX_LENGTH;
        }
        Com_ReadCtrl_(handle);
        break;
    case RX_LENGTH:
        // Get data
        handle->dataLength_ = handle->ctrlByte_;
        HAL_CRC_Accumulate(handle->hcrc, (uint32_t *)&handle->ctrlByte_, 1);
        if(handle->dataLength_) {
            handle->rxState_ = RX_DATA;
            Com_ReadData_(handle);
        } else {
            handle->rxState_ = RX_CRC;
            Com_ReadCtrl_(handle);
        }
        break;
    case RX_DATA:
        // Get CRC
        HAL_CRC_Accumulate(handle->hcrc, (uint32_t *)&handle->dataBuf_, handle->dataLength_);
        handle->rxState_ = RX_CRC;
        Com_ReadCtrl_(handle);
        break;
    case RX_CRC:
        // Check CRC and process
        uint8_t crc = HAL_CRC_Accumulate(handle->hcrc, (uint32_t *)&handle->ctrlByte_, 1);
        // Send Nack if wrong crc
        if (crc) {
            Com_SendAck_(handle, 0);
        } else {
            if(handle->header_.rw) {
                Com_Process_Write_(handle);
            } else {
                Com_Process_Read_(handle);
            }
        }
        handle->rxState_ = RX_START;
        Com_ReadCtrl_(handle);
        break;
    }
}