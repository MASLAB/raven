#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

union Pid_Config {
    struct {
        float kp;
        float ki;
        float kd;
    };
    uint8_t bytes[12];
};

struct Pid_Handle {
    // configuration
    union Pid_Config config;

    // internal
    float target;

    float errPrev;
    float errInt;
};

void Pid_Init(struct Pid_Handle* handle);

float Pid_Update(struct Pid_Handle* handle, float pos);

// pid reset is same as init
inline void Pid_Reset(struct Pid_Handle* handle) {
    Pid_Init(handle);
}

#endif