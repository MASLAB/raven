#ifndef INC_DRV8874_H_
#define INC_DRV8874_H_

#include "main.h"

struct Drv8874_Handle {
    // configuration
    TIM_HandleTypeDef* vTim; // voltage
    uint32_t vChan;

    TIM_HandleTypeDef* cTim; // current
    uint32_t cChan;

    void (*eFunc)(uint8_t); // enable
    void (*dFunc)(uint8_t); // direction
};

void Drv8874_Init(struct Drv8874_Handle* handle);

void Drv8874_SetEnable(struct Drv8874_Handle* handle, uint8_t en);
void Drv8874_SetDirection(struct Drv8874_Handle* handle, uint8_t dir);

void Drv8874_SetVoltage(struct Drv8874_Handle* handle, uint8_t voltage);
void Drv8874_SetCurrent(struct Drv8874_Handle* handle, uint8_t current);


#endif