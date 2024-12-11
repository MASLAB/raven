#include "encoder.h"

void Encoder_Init(struct Encoder_Handle* handle) {
	handle->aLast = 0;
	handle->bLast = 0;

	handle->pos = 0;
}

void Encoder_Update(struct Encoder_Handle* handle) {
	const uint8_t aState = HAL_GPIO_ReadPin(handle->aPort, handle->aPin);
	const uint8_t bState = HAL_GPIO_ReadPin(handle->bPort, handle->bPin);

	const uint8_t aDiff = aState^handle->aLast;
	const uint8_t bDiff = bState^handle->bLast;

	if (aDiff^bDiff) {
		const uint8_t idir = aDiff ^ aState ^ bState ^ handle->dir;
		handle->pos += (idir<<1);
		handle->pos -= 1;
	}
	handle->aLast = aState;
	handle->bLast = bState;
}
