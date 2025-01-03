#include "drv8874.h"

void Drv8874_Init(struct Drv8874_Handle* handle) {
    HAL_TIM_PWM_Start(handle->vTim, handle->vChan);
    HAL_TIM_PWM_Start(handle->cTim, handle->cChan);

    __HAL_TIM_SET_COMPARE(handle->vTim, handle->vChan, 0);
    __HAL_TIM_SET_COMPARE(handle->cTim, handle->cChan, 0);

	handle->eFunc(0);
	handle->dFunc(0);
}

void Drv8874_SetEnable(struct Drv8874_Handle* handle, bool en) {
	handle->eFunc(en);
}

void Drv8874_SetDirection(struct Drv8874_Handle* handle, bool dir) {
	handle->dFunc(dir);
}

void Drv8874_SetVoltage(struct Drv8874_Handle* handle, uint16_t voltage) {
	__HAL_TIM_SET_COMPARE(handle->vTim, handle->vChan, voltage);
}

void Drv8874_SetCurrent(struct Drv8874_Handle* handle, uint16_t current) {
	__HAL_TIM_SET_COMPARE(handle->cTim, handle->cChan, current);
}