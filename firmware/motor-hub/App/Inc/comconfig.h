#ifndef INC_COMCONFIG_H_
#define INC_COMCONFIG_H_

#include "stdbool.h"
#include "stdint.h"

/***
 * Message format
 * Start: 1 byte
 *   [7-0] start byte
 * Header: 1 byte
 *   [7] 0: read, 1: write
 *   [6-0] header type
 * Length: 1 byte
 *   [7-0] data length
 * Data bytes : any number of bytes
 *    ...
 * CRC: 1 byte
 *   [7-0] 8-bit CRC
 */

#define COM_BUF_SIZE 16 // Longest is PID (12 bytes data + start, pcf, header, crc)

typedef enum __attribute__((packed)) {
    HDR_NACK,
    HDR_ACK,
    HDR_REPLY,
    HDR_MOTOR_CMD,
    HDR_MOTOR_MODE,
    HDR_MOTOR_PID,
    HDR_SERVO_VALUE,
    HDR_ENCODER_VALUE,
    HDR_ERROR,
    NUM_HDR,
} Header_Type_t;

// Packing all the messages for concise messages from comm module
#define TYPEDEF_PACKED typedef struct __attribute__((packed))

TYPEDEF_PACKED {
    uint8_t channel;
    float command;
} Motor_Cmd_Msg_t;

typedef enum __attribute__((packed)) {
    MOTOR_MODE_NULL,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_POSITION,
} Motor_Mode_t;

TYPEDEF_PACKED {
    uint8_t channel;
    Motor_Mode_t mode;
} Motor_Mode_Msg_t;


TYPEDEF_PACKED {
    float p;
    float i;
    float d;
} Motor_PID_t;

TYPEDEF_PACKED {
    uint8_t channel;
    Motor_PID_t pid;
} Motor_PID_Msg_t;

TYPEDEF_PACKED {
    uint8_t channel;
    uint16_t timer_val;
} Servo_Value_Msg_t;

// Will only request encoder value
// Will only request error value

#endif