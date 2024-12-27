#include "app.h"
#include "main.h"
#include "sipo.h"
#include "encoder.h"
#include "drv8874.h"
#include "com.h"

#define ENCODER(num) {.aPort = A##num##_GPIO_Port, .aPin = A##num##_Pin, .bPort = B##num##_GPIO_Port, .bPin = B##num##_Pin}
#define CONFIG(name,index) static void name(uint8_t state){if(state){sipo.data|=1<<index;}else{sipo.data&=~(1<<index);}}

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

struct Sipo_Handle sipo = {.clkPort = RCLK_GPIO_Port, .clkPin = RCLK_Pin, .dataPort = RDAT_GPIO_Port, .dataPin = RDAT_Pin, .loadPort = RSV_GPIO_Port, .loadPin = RSV_Pin};

// RECIEVE
// 0:servos (4)
// 1:motor (5) (mode, request, voltage, current)
// 2:PIDs (5) (P, I, D)
// hearbeat

// SEND
// 0:current (5)
// 1:encoders (5)

// DATA FORMAT
// message type (3bit)
// channel (3bit)
// parameter (2bit)
// data (0-31 bytes)

static struct Message {
    void (*fn)(uint8_t*);
    uint8_t len;
};

static void pwm(uint8_t* data) {
    UNUSED(data);
}

static void motor(uint8_t* data) {
    UNUSED(data);
}

#define MCOUNT 2
static struct Message messages[MCOUNT] = {{.fn = &pwm, .len = 2}, {.fn = &motor, .len = 2}};

static void receive(uint8_t* data, uint8_t len) {
    if (!len) {
        return; // error
    }
    const uint8_t mType = data[0];
    if (mType >= MCOUNT) {
        return; // error
    }
    if (messages[mType].len != len) {
        return; // error
    }
    (messages[mType].fn)(data);
}

struct Com_Handle com = {.huart = &huart3, .hdma = &hdma_usart3_rx, .hcrc = &hcrc, .receive = &receive, .sByte = 0xAA, .maxRetries = 5};

struct Encoder_Handle encoders[5] = {
    ENCODER(1),
    ENCODER(2),
    ENCODER(3),
    ENCODER(4),
    ENCODER(5),
};

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

static uint16_t adc1Data[3] = {0};
static uint16_t adc2Data[4] = {0};

void App_Init(void) {
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1Data, 3);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2Data, 4);

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
}

void App_Update(void) {
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    UNUSED(huart);
    Com_Receive(&com, size);
}