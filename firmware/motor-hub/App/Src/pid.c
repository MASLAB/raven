#include "pid.h"

void Pid_Init(struct Pid_Handle* handle) {
    handle->target = 0;

    handle->errPrev = 0;
    handle->errInt = 0;
}

float Pid_Update(struct Pid_Handle* handle, float pos) {
    const float err = handle->target - pos;

    const float p = handle->kp*err;

    handle->errInt += err;
	const float i = handle->ki*handle->errInt;

	const float d = handle->kd*(err - handle->errPrev);
	handle->errPrev = err;

	return p+i+d;
}
