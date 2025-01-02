#include "app.h"
#include "string.h"
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
    {.config = {.kp = 0, .ki = 0, .kd = 0}},
    {.config = {.kp = 0, .ki = 0, .kd = 0}},
    {.config = {.kp = 0, .ki = 0, .kd = 0}},
    {.config = {.kp = 0, .ki = 0, .kd = 0}},
    {.config = {.kp = 0, .ki = 0, .kd = 0}},
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

// com functions and initialization //
#define MESSAGES 9
#define MAX_CHANNEL 5

static struct Com_Handle com;

static void com_callback(UART_HandleTypeDef* huart) {
    UNUSED(huart);
    Com_Callback(&com);
}

static void com_send (uint8_t* data, uint8_t len) {
    while ((HAL_UART_GetState(&huart3) & 1) != 0);
    HAL_UART_Transmit_DMA(&huart3, data, len);
}

static void com_request (uint8_t* data, uint8_t len) {
    HAL_UART_Receive_DMA(&huart3, data, len);
}

static enum Message_RW {
    Message_Write = 0u,
    Message_Read = 1u,
};

static struct RegisterBlock {
    uint8_t** regs;

    uint8_t size;
    
    // for special updates
    void (*writeAction)(uint8_t);
};

static enum Message_Type {
    // servos
    Message_Servo = 0u, // rw

    // motors
    Message_Mode = 1u, // rw
    Message_PID = 2u, // rw
    Message_Target = 3u, // rw
    Message_Current = 4u, // rw
    Message_Voltage = 5u, // rw
    Message_Encoder = 6u, // rw
    Message_MeasCurrent = 7u, // r

    // battery
    Message_Battery = 8u, // r
};

static void writeMode(uint8_t chan) {
    //TODO
}

static void writeEncoder(uint8_t chan) {
    //TODO
}

// register initialization happens in Init to get pointers
static uint8_t* servoRegs[MAX_CHANNEL] = {0};
static uint8_t* modeRegs[MAX_CHANNEL] = {0};
static uint8_t* pidRegs[MAX_CHANNEL] = {0};
static uint8_t* targetRegs[MAX_CHANNEL] = {0};
static uint8_t* currentRegs[MAX_CHANNEL] = {0};
static uint8_t* voltageRegs[MAX_CHANNEL] = {0};
static uint8_t* encoderRegs[MAX_CHANNEL] = {0};
static uint8_t* measCurRegs[MAX_CHANNEL] = {0};
static uint8_t* measBatRegs[MAX_CHANNEL] = {0};

// index corresponds to message type
const static struct RegisterBlock regBlocks[MESSAGES] = {
    {.regs = &servoRegs, .size = 2, .writeAction = NULL}, // servo (chan 0-4)
    {.regs = &modeRegs, .size = 1, .writeAction = &writeMode}, // mode
    {.regs = &pidRegs, .size = 12, .writeAction = NULL}, // PID
    {.regs = &targetRegs, .size = 4, .writeAction = NULL}, // PID target
    {.regs = &currentRegs, .size = 2, .writeAction = NULL}, // current limit
    {.regs = &voltageRegs, .size = 2, .writeAction = NULL}, // voltage
    {.regs = &encoderRegs, .size = 4, .writeAction = &writeEncoder}, // encoder
    {.regs = &measCurRegs, .size = 2, .writeAction = NULL}, // measured current
    {.regs = &measBatRegs, .size = 2, .writeAction = NULL}, // measured battery voltage (chan 0 only)
};

static union Message_Header {
    struct {
        enum Message_RW rw : 1;
        enum Message_Type type : 4;
        uint8_t channel : 3;
    };
    uint8_t byte;
};

static uint8_t com_parse (uint8_t* data, uint8_t len) {
    if (!len) return 0; // invalid header

    const union Message_Header header = {.byte = data[0]};
    if (header.type >= MESSAGES) return 0; // invalid message

    const uint8_t chan = header.channel;
    if (chan >= MAX_CHANNEL) return 0; // invalid channel

    const bool read = header.rw == Message_Read;

    const struct RegisterBlock block = regBlocks[header.type];
    uint8_t* ptr = &block.regs[chan*block.size];
    if (read) {
        if (ptr) {
            memcpy(data, ptr, block.size);
        }else {
            memset(data, 0, block.size);
        }
        return block.size;
    }
    // 1 byte shift because header
    if (ptr) memcpy(ptr, &data[1], block.size);

    if (block.writeAction) (*block.writeAction)(chan);
    return 0;
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

    // servo pointers
    for (uint8_t i = 0; i < 4; i++) {
        //TODO
    }
    // motor pointers
    for (uint8_t i = 0; i < 5; i++) {
        //TODO
    }
    // bat pointer
    measBatRegs[0] = (uint8_t*)vbat;

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