#include "app.h"
#include "main.h"
#include "sipo.h"
#include "encoder.h"
#include "drv8874.h"
#include "com.h"
#include "servo.h"
#include "pid.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern CRC_HandleTypeDef hcrc;

// shift register
// automatically handled with timer
struct Sipo_Handle sipo = {.clkPort = RCLK_GPIO_Port, .clkPin = RCLK_Pin, .dataPort = RDAT_GPIO_Port, .dataPin = RDAT_Pin, .loadPort = RSV_GPIO_Port, .loadPin = RSV_Pin};

#define ENCODER(num) {.aPort = A##num##_GPIO_Port, .aPin = A##num##_Pin, .bPort = B##num##_GPIO_Port, .bPin = B##num##_Pin}
struct Encoder_Handle encoders[5] = {
    ENCODER(1),
    ENCODER(2),
    ENCODER(3),
    ENCODER(4),
    ENCODER(5),
};

// velocities for vel PID mode
static float vels[5] = {0};
static int32_t lastPos[5] = {0};

// functions for dir and en
// some are through shift register and some are GPIO
#define CONFIG(name,index) static void name(uint8_t state){if(state){sipo.data|=1<<index;}else{sipo.data&=~(1<<index);}}
CONFIG(dir1, 0)
CONFIG(en1, 1)
CONFIG(dir2, 2)
CONFIG(en2, 3)
CONFIG(dir3, 4)
CONFIG(en3, 5)
CONFIG(dir4, 6)
CONFIG(en4, 7)

static void dir5(uint8_t dir) {
    HAL_GPIO_WritePin(PH5_GPIO_Port, PH5_Pin, dir);
}
static void en5(uint8_t en) {
    HAL_GPIO_WritePin(SL5_GPIO_Port, SL5_Pin, en);
}

struct Drv8874_Handle motors[5] = {
    {.vTim = &htim2, .vChan = TIM_CHANNEL_2, .cTim = &htim4, .cChan = TIM_CHANNEL_4, .eFunc = &en1, .dFunc = &dir1},
    {.vTim = &htim2, .vChan = TIM_CHANNEL_3, .cTim = &htim2, .cChan = TIM_CHANNEL_4, .eFunc = &en2, .dFunc = &dir2},
    {.vTim = &htim2, .vChan = TIM_CHANNEL_1, .cTim = &htim3, .cChan = TIM_CHANNEL_1, .eFunc = &en3, .dFunc = &dir3},
    {.vTim = &htim3, .vChan = TIM_CHANNEL_4, .cTim = &htim3, .cChan = TIM_CHANNEL_3, .eFunc = &en4, .dFunc = &dir4},
    {.vTim = &htim8, .vChan = TIM_CHANNEL_1, .cTim = &htim15, .cChan = TIM_CHANNEL_2, .eFunc = &en5, .dFunc = &dir5},
};

struct Pid_Handle pids[5] = {
    {.kp = 0, .ki = 0, .kd = 0},
    {.kp = 0, .ki = 0, .kd = 0},
    {.kp = 0, .ki = 0, .kd = 0},
    {.kp = 0, .ki = 0, .kd = 0},
    {.kp = 0, .ki = 0, .kd = 0},
};

enum MotorMode {
    MotorModeDirect = 0u, // direct control (for current limiting push into objects or smth)
    MotorModePos = 1u, // position pid
    MotorModeVel = 2u, // velocity pid
};

// begin all in direct mode
enum MotorMode motorModes[5] = {0};

struct Servo_Handle servos[4] = {
    {.tim = &htim1, .chan = TIM_CHANNEL_4},
    {.tim = &htim16, .chan = TIM_CHANNEL_1},
    {.tim = &htim1, .chan = TIM_CHANNEL_3},
    {.tim = &htim1, .chan = TIM_CHANNEL_2},
};

// dma buffers for ADC
static uint16_t adc1Data[2] = {0};
static uint16_t adc2Data[4] = {0};

// pointers to proper values
static uint16_t* vbat = &adc1Data[1];
static uint16_t* currents[5] = {&adc1Data[0], &adc2Data[3], &adc2Data[0], &adc2Data[1], &adc2Data[2]};

// Com handle
static Com_Handle_t com = {.huart = &huart3, .hcrc = &hcrc, .sByte = 0xAA};
// Com callbacks
// Reads
static Com_Reply_t* MotorCmdRead(uint8_t* message, uint8_t length);
static Com_Reply_t* MotorModeRead(uint8_t* message, uint8_t length);
static Com_Reply_t* MotorPIDRead(uint8_t* message, uint8_t length);
static Com_Reply_t* EncoderValueRead(uint8_t* message, uint8_t length);
static Com_Reply_t* ErrorRead(uint8_t* message, uint8_t length);
// Writes
static bool MotorCmdWrite(uint8_t* message, uint8_t length);
static bool MotorModeWrite(uint8_t* message, uint8_t length);
static bool MotorPIDWrite(uint8_t* message, uint8_t length);
static bool ServoValueWrite(uint8_t* message, uint8_t length);

void App_Init(void) {
    // HAL_TIM_Base_Start_IT(&htim6);
    // HAL_TIM_Base_Start_IT(&htim7);

    // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1Data, 2);
    // HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2Data, 4);

    // Sipo_Init(&sipo);
    
    // Encoder_Init(&encoders[0]);
    // Encoder_Init(&encoders[1]);
    // Encoder_Init(&encoders[2]);
    // Encoder_Init(&encoders[3]);
    // Encoder_Init(&encoders[4]);

    // Drv8874_Init(&motors[0]);
    // Drv8874_Init(&motors[1]);
    // Drv8874_Init(&motors[2]);
    // Drv8874_Init(&motors[3]);
    // Drv8874_Init(&motors[4]);

    // Servo_Init(&servos[0]);
    // Servo_Init(&servos[1]);
    // Servo_Init(&servos[2]);
    // Servo_Init(&servos[3]);

    // Pid_Init(&pids[0]);
    // Pid_Init(&pids[1]);
    // Pid_Init(&pids[2]);
    // Pid_Init(&pids[3]);
    // Pid_Init(&pids[4]);

    Com_Init(&com);
    Com_RegisterReadCallback(&com, HDR_MOTOR_CMD, &MotorCmdRead);
    Com_RegisterReadCallback(&com, HDR_MOTOR_MODE, &MotorModeRead);
    Com_RegisterReadCallback(&com, HDR_MOTOR_PID, &MotorPIDRead);
    Com_RegisterReadCallback(&com, HDR_ENCODER_VALUE, &EncoderValueRead);
    Com_RegisterReadCallback(&com, HDR_ERROR, &ErrorRead);

    Com_RegisterWriteCallback(&com, HDR_MOTOR_CMD, &MotorCmdWrite);
    Com_RegisterWriteCallback(&com, HDR_MOTOR_MODE, &MotorModeWrite);
    Com_RegisterWriteCallback(&com, HDR_MOTOR_PID, &MotorPIDWrite);
    Com_RegisterWriteCallback(&com, HDR_SERVO_VALUE, &ServoValueWrite);
}

static void check_vbat(void) {
    // value after divider
    const uint16_t adcbat = *vbat;

    if (adcbat < 1200) {
        // unlatch FET
        HAL_GPIO_WritePin(PWR_GPIO_Port, PWR_Pin, 0);
    }
}

static inline void update_motor(uint8_t chan) {
    if (motorModes[chan] == MotorModePos) {
        const int16_t out = Pid_Update(&pids[chan], (float)encoders[chan].pos);
        const uint8_t dir = out < 0;
        Drv8874_SetDirection(&motors[chan], dir);
        if (dir) {
            Drv8874_SetVoltage(&motors[chan], (uint8_t)(-out));
        }else {
            Drv8874_SetVoltage(&motors[chan], (uint8_t)out);
        }
    }else if (motorModes[chan] == MotorModeVel) {
        const int16_t out = Pid_Update(&pids[chan], vels[chan]);
        const uint8_t dir = out < 0;
        Drv8874_SetDirection(&motors[chan], dir);
        if (dir) {
            Drv8874_SetVoltage(&motors[chan], (uint8_t)(-out));
        }else {
            Drv8874_SetVoltage(&motors[chan], (uint8_t)out);
        }
    }
}

void App_Update(void) {
    check_vbat();
    
    // TODO calculate delta us with a timer
    uint32_t dut = 1000;
    for (uint8_t i = 0; i < 5; i++) {
        vels[i] = 1000000.f*(float)(encoders[i].pos - lastPos[i])/((float)dut);
        lastPos[i] = encoders[i].pos;
    }

    update_motor(0);
    update_motor(1);
    update_motor(2);
    update_motor(3);
    update_motor(4);

    HAL_Delay(1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim6) {
        Sipo_Update(&sipo);
    }else {//htim7
        Encoder_Update(&encoders[0]);
        Encoder_Update(&encoders[1]);
        Encoder_Update(&encoders[2]);
        Encoder_Update(&encoders[3]);
        Encoder_Update(&encoders[4]);
    }
}

// Com callbacks
// Reads
Com_Reply_t* MotorCmdRead(uint8_t* message, uint8_t length) {
    static float command;
    static Com_Reply_t reply = {
        .length = sizeof(float), // 1 float value for command
        .data = (void*) &command,
    };

    uint8_t channel = &message;
    command = 0.0; // TODO: Modify command according to channel
    
    return &reply;
}

Com_Reply_t* MotorModeRead(uint8_t* message, uint8_t length) {
    static Motor_Mode_t mode;
    static Com_Reply_t reply = {
        .length = sizeof(Motor_Mode_t), // 1 byte of mode
        .data = (void*) &mode,
    };
    
    uint8_t channel = &message;
    mode = MOTOR_MODE_NULL; // TODO: Modify mode according to channel

    return &reply;
}

Com_Reply_t* MotorPIDRead(uint8_t* message, uint8_t length) {
    static Motor_PID_t pid;
    static Com_Reply_t reply = {
        .length = sizeof(Motor_PID_t), // 3 values of PID
        .data = (void*) &pid,
    };
    
    uint8_t channel = &message;
    // TODO: Modify pid according to channel
    pid.p = 1;
    pid.i = 2;
    pid.d = 3;
    
    return &reply;
}

Com_Reply_t* EncoderValueRead(uint8_t* message, uint8_t length) {
    static uint32_t value;
    static Com_Reply_t reply = {
        .length = sizeof(uint32_t), // 1 encoder value
        .data = (void*) &value,
    };

    uint8_t channel = &message;
    value = 0; // TODO: Modify the reply according to channel

    return &reply;
}

Com_Reply_t* ErrorRead(uint8_t* message, uint8_t length) {
    static Com_Reply_t reply;
    // TODO: Modify the reply
    return &reply;
}

// Writes
bool MotorCmdWrite(uint8_t* message, uint8_t length) {
    Motor_Cmd_Msg_t* msg = (Motor_Cmd_Msg_t*) message;
    return true;
}

bool MotorModeWrite(uint8_t* message, uint8_t length) {
    Motor_Mode_Msg_t* msg = (Motor_Mode_Msg_t*) message;
    // TODO: Do something about it
    return true;
}

bool MotorPIDWrite(uint8_t* message, uint8_t length) {
    Motor_PID_Msg_t* msg = (Motor_PID_Msg_t*) message;
    // TODO: Do something about it
    return true;
}

bool ServoValueWrite(uint8_t* message, uint8_t length) {
    Servo_Value_Msg_t* msg = (Servo_Value_Msg_t*) message;
    // TODO: Do something about it
    return true;
}