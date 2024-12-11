#include "app.h"
#include "main.h"
#include "sipo.h"
#include "encoder.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim15;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

int count = 0;
int count2 = 0;
volatile int conv = 0;
struct Sipo_Handle sipo = {.clkPort = RCLK_GPIO_Port, .clkPin = RCLK_Pin, .dataPort = RDAT_GPIO_Port, .dataPin = RDAT_Pin, .loadPort = RSV_GPIO_Port, .loadPin = RSV_Pin};
struct Encoder_Handle encoder5 = {.aPort = A5_GPIO_Port, .aPin = A5_Pin, .bPort = B5_GPIO_Port, .bPin = B5_Pin, .dir = 0};
uint16_t adc1Data[2] = {0};
uint16_t adc2Data[4] = {0};

void App_Init(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(SL5_GPIO_Port, SL5_Pin, 1);
    HAL_GPIO_WritePin(PH5_GPIO_Port, PH5_Pin, 1);

    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1Data, 2);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2Data, 4);

    Sipo_Init(&sipo);
    Encoder_Init(&encoder5);
}

void App_Update(void) {
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim6) {
        Sipo_Update(&sipo);
        count++;
        if (count == 200) {
            count2++;
            sipo.data = encoder5.pos&0xFF;
            if (count2 == 4) {
                // HAL_GPIO_TogglePin(PH5_GPIO_Port, PH5_Pin);
                count2 = 0;
            }
            if (conv) {
                conv = 0;
                // HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2Data, 4);
            }
            // hsipo.data++;
            count = 0;
        }
    }else {//htim7
        Encoder_Update(&encoder5);
    }
}