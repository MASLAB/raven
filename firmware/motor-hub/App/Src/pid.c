#include "pid.h"

void Pid_Init(struct Pid_Handle* handle) {
    handle->target = 0;

    handle->errPrev = 0;
    handle->errInt = 0;
}

float Pid_Update(struct Pid_Handle* handle, float pos) {
    const float err = handle->target - pos;

    const float p = handle->config.kp*err;

    handle->errInt += err;
	const float i = handle->config.ki*handle->errInt;

	const float d = handle->config.kd*(err - handle->errPrev);
	handle->errPrev = err;

	return p+i+d;
}
