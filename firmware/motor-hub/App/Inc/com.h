#ifndef INC_COM_H_
#define INC_COM_H_

#include "main.h"

#include "comconfig.h"

typedef enum {
    RX_START,
    RX_HEADER,
    RX_LENGTH,
    RX_DATA,
    RX_CRC,
} Rx_State_t;

typedef union {
    struct {
        uint8_t header : 7;
        bool rw : 1;
    };
    uint8_t value;
} Com_Header_t;

typedef struct {
    uint8_t length;
    void* data;
} Com_Reply_t;

typedef struct {
	// configuration
    // Public
    UART_HandleTypeDef* huart; // UART handle
    CRC_HandleTypeDef* hcrc; // CRC handle
    uint8_t sByte; // Start byte

    // Private
    uint8_t txBuf_[COM_BUF_SIZE];
    Rx_State_t rxState_; // State for reading
    uint8_t ctrlByte_; // Control byte (start, header, length, crc)
    Com_Header_t header_; // Current control header
    uint8_t dataLength_; // Length of data
    uint8_t dataBuf_[COM_BUF_SIZE-3]; // Minus start, pcf, and crc
    // Callbacks
    Com_Reply_t* (*readCallbacks_[NUM_HDR])(void* message, uint8_t length);
    bool (*writeCallbacks_[NUM_HDR])(void* message, uint8_t length);
} Com_Handle_t;

/**
 * @brief Initialize communication interface
 * @param handle Pointer to a Com_Handle_t object
 * The handle should be initialized with:
 * huart: HAL UART_HandleTypeDef 
 * hcrc: HAL_CRC_HandleTypeDef
 * sByte: Start byte
 */
void Com_Init(Com_Handle_t* handle);

/**
 * @brief Register read callback
 * @param handle Pointer to a Com_Handle_t object
 * @param header_type The expected header for read
 * @param callback Pointer to callback function
 * The callback function should return a pointer to a Com_Reply_t
 * that specifies the length of the data and the pointer to the data.
 */
void Com_RegisterReadCallback(
    Com_Handle_t* handle,
    Header_Type_t header_type,
    Com_Reply_t* (*callback)(void* message, uint8_t length)
);

/**
 * @brief Register write callback
 * @param handle Pointer to a Com_Handle_t object
 * @param header_type The expected header for read
 * @param callback Pointer to callback function
 * The callback function should return true if write operation is 
 * successful and false otherwise
 */
void Com_RegisterWriteCallback(
    Com_Handle_t* handle,
    Header_Type_t header_type,
    bool (*callback)(void* message, uint8_t length)
);

#endif