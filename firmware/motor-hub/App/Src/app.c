#include "app.h"
#include "string.h"
#include "main.h"
#include "sipo.h"
#include "encoder.h"
#include "drv8874.h"
#include "com.h"
#include "servo.h"
#include "pid.h"
#include "parser.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern UART_HandleTypeDef huart3;
extern CRC_HandleTypeDef hcrc;

// shift register
// automatically handled with timer
static struct Sipo_Handle sipo = {.clkPort = RCLK_GPIO_Port, .clkPin = RCLK_Pin, .dataPort = RDAT_GPIO_Port, .dataPin = RDAT_Pin, .loadPort = RSV_GPIO_Port, .loadPin = RSV_Pin};

#define ENCODER(num) {.aPort = A##num##_GPIO_Port, .aPin = A##num##_Pin, .bPort = B##num##_GPIO_Port, .bPin = B##num##_Pin}
static struct Encoder_Handle encoders[5] = {
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

static struct Drv8874_Handle motors[5] = {
    {.vTim = &htim2, .vChan = TIM_CHANNEL_2, .cTim = &htim4, .cChan = TIM_CHANNEL_4, .eFunc = &en1, .dFunc = &dir1},
    {.vTim = &htim2, .vChan = TIM_CHANNEL_3, .cTim = &htim2, .cChan = TIM_CHANNEL_4, .eFunc = &en2, .dFunc = &dir2},
    {.vTim = &htim2, .vChan = TIM_CHANNEL_1, .cTim = &htim3, .cChan = TIM_CHANNEL_1, .eFunc = &en3, .dFunc = &dir3},
    {.vTim = &htim3, .vChan = TIM_CHANNEL_4, .cTim = &htim3, .cChan = TIM_CHANNEL_3, .eFunc = &en4, .dFunc = &dir4},
    {.vTim = &htim8, .vChan = TIM_CHANNEL_1, .cTim = &htim15, .cChan = TIM_CHANNEL_2, .eFunc = &en5, .dFunc = &dir5},
};

static struct Pid_Handle pids[5] = {
    {.kp = 0, .ki = 0, .kd = 0},
    {.kp = 0, .ki = 0, .kd = 0},
    {.kp = 0, .ki = 0, .kd = 0},
    {.kp = 0, .ki = 0, .kd = 0},
    {.kp = 0, .ki = 0, .kd = 0},
};

static enum MotorMode {
    MotorModeDisable = 0u,
    MotorModeDirect = 1u, // direct control (for current limiting push into objects or smth)
    MotorModePos = 2u, // position pid
    MotorModeVel = 3u, // velocity pid
};

// all begin in direct mode
static enum MotorMode motorModes[5] = {0};

static struct Servo_Handle servos[4] = {
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

// parse functions
static uint8_t servo_read(uint8_t*, uint8_t);
static uint8_t mode_read(uint8_t*, uint8_t);
static uint8_t pid_read(uint8_t*, uint8_t);
static uint8_t target_read(uint8_t*, uint8_t);
static uint8_t voltage_read(uint8_t*, uint8_t);
static uint8_t current_read(uint8_t*, uint8_t);
static uint8_t encoder_read(uint8_t*, uint8_t);
static uint8_t measCur_read(uint8_t*, uint8_t);
static uint8_t measBat_read(uint8_t*, uint8_t);

static void servo_write(uint8_t*, uint8_t);
static void mode_write(uint8_t*, uint8_t);
static void pid_write(uint8_t*, uint8_t);
static void target_write(uint8_t*, uint8_t);
static void voltage_write(uint8_t*, uint8_t);
static void current_write(uint8_t*, uint8_t);
static void encoder_write(uint8_t*, uint8_t);

static uint8_t (*readFns[9])(uint8_t*, uint8_t) = {
    &servo_read,
    &mode_read,
    &pid_read,
    &target_read,
    &voltage_read,
    &current_read,
    &encoder_read,
    &measCur_read,
    &measBat_read,
};
static void (*writeFns[9])(uint8_t*, uint8_t) = {
    &servo_write,
    &mode_write,
    &pid_write,
    &target_write,
    &voltage_write,
    &current_write,
    &encoder_write,
    NULL,
    NULL,
};

static struct Parser_Handle parser = {
    .writes = &writeFns,
    .reads = &readFns,
    .len = 9,
    .typeBits = 4, // max 16 types
};

static struct Com_Handle com;

static void com_callback(UART_HandleTypeDef* huart) {
    UNUSED(huart);
    Com_Handler(&com);
}

static void com_send (uint8_t* data, uint8_t len) {
    while ((HAL_UART_GetState(&huart3) & 1) != 0);
    HAL_UART_Transmit_DMA(&huart3, data, len);
}

static void com_request (uint8_t* data, uint8_t len) {
    HAL_UART_Receive_DMA(&huart3, data, len);
}

static uint8_t com_parse (uint8_t* data, uint8_t len) {
    return Parser_Handler(&parser, data, len);
}

static struct Com_Handle com = {
    .hcrc = &hcrc,
    .sByte = 0xAA,
    .maxData = 12,
    .send = &com_send,
    .request = &com_request,
    .parse = &com_parse,
};

void App_Init(void) {
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim17);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1Data, 2);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2Data, 4);

    HAL_UART_RegisterCallback(&huart3, HAL_UART_RX_COMPLETE_CB_ID, com_callback);
    HAL_UART_Init(&huart3);

    Sipo_Init(&sipo);
    
    Encoder_Init(&encoders[0]);
    Encoder_Init(&encoders[1]);
    Encoder_Init(&encoders[2]);
    Encoder_Init(&encoders[3]);
    Encoder_Init(&encoders[4]);

    Drv8874_Init(&motors[0]);
    Drv8874_Init(&motors[1]);
    Drv8874_Init(&motors[2]);
    Drv8874_Init(&motors[3]);
    Drv8874_Init(&motors[4]);

    Servo_Init(&servos[0]);
    Servo_Init(&servos[1]);
    Servo_Init(&servos[2]);
    Servo_Init(&servos[3]);

    Pid_Init(&pids[0]);
    Pid_Init(&pids[1]);
    Pid_Init(&pids[2]);
    Pid_Init(&pids[3]);
    Pid_Init(&pids[4]);

    Com_Init(&com);
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
    HAL_Delay(50);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim6) {
        Sipo_Update(&sipo);
    }else if (htim == &htim7){
        // TODO figure this out when motors arrive
        Encoder_Update(&encoders[0]);
        Encoder_Update(&encoders[1]);
        Encoder_Update(&encoders[2]);
        Encoder_Update(&encoders[3]);
        Encoder_Update(&encoders[4]);
    } else { // tim17
        update_motor(0);
        update_motor(1);
        update_motor(2);
        update_motor(3);
        update_motor(4);
    }
}

static uint8_t servo_read(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    const uint16_t val = (chan < 4) ? Servo_Read(&servos[chan]) : 0;
    memcpy(data, val, 2);
    return 2;
}
static uint8_t mode_read(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    data[0] = (chan < 5) ? motorModes[chan] : 0;
    return 1;
}
static uint8_t pid_read(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    //TODO
    return 12;
}
static uint8_t target_read(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    //TODO
    return 4;
}
static uint8_t voltage_read(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    //TODO
    return 2;
}
static uint8_t current_read(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    //TODO
    return 2;
}
static uint8_t encoder_read(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    //TODO
    return 4;
}
static uint8_t measCur_read(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    //TODO
    return 2;
}
static uint8_t measBat_read(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    const uint16_t val = chan ? 0 : *vbat;
    memcpy(data, &val, 2);
    return 2;
}

static void servo_write(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    if (chan < 4) {
        uint16_t val;
        memcpy(&val, &data[1], 2);
        Servo_Write(&servos[chan], val);
    }
}
static void mode_write(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    if (chan < 5) {
        motorModes[chan] = data[1];
    }
}
static void pid_write(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    if (chan < 5) {
        //TODO
    }
}
static void target_write(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    if (chan < 5) {
        float val;
        memcpy(&val, &data[1], 4);
        pids[chan].target = val;
    }
}
static void voltage_write(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    if (chan < 5) {
        uint16_t val;
        memcpy(&val, &data[1], 2);
        Drv8874_SetVoltage(&motors[chan], val);
    }
}
static void current_write(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    if (chan < 5) {
        uint16_t val;
        memcpy(&val, &data[1], 2);
        Drv8874_SetCurrent(&motors[chan], val);
    }
}
static void encoder_write(uint8_t* data, uint8_t len) {
    const uint8_t chan = data[0]&3;
    //TODO
}