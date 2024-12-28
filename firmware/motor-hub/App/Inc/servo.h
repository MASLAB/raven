#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

struct Servo_Handle {
    // configuration
    TIM_HandleTypeDef* tim;
    uint32_t chan;
};

void Servo_Init(struct Servo_Handle* handle);

void Servo_Move(struct Servo_Handle* handle, uint16_t pos);

#endif