#ifndef INC_DRV8874_H_
#define INC_DRV8874_H_

#include "main.h"
#include "stdbool.h"

struct Drv8874_Handle {
    // configuration
    TIM_HandleTypeDef* vTim; // voltage
    uint32_t vChan;

    TIM_HandleTypeDef* cTim; // current
    uint32_t cChan;

    void (*eFunc)(bool); // enable
    void (*dFunc)(bool); // direction

    //internal
    bool dir;
};

void Drv8874_Init(struct Drv8874_Handle* handle);

void Drv8874_SetEnable(struct Drv8874_Handle* handle, bool en);

void Drv8874_SetDirection(struct Drv8874_Handle* handle, bool dir);
bool Drv8874_GetDirection(struct Drv8874_Handle* handle);

void Drv8874_SetVoltage(struct Drv8874_Handle* handle, uint16_t voltage);
void Drv8874_SetCurrent(struct Drv8874_Handle* handle, uint16_t current);

uint16_t Drv8874_GetVoltage(struct Drv8874_Handle* handle);
uint16_t Drv8874_GetCurrent(struct Drv8874_Handle* handle);

#endif