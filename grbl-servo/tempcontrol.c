#include <stm32f4xx_hal.h>
#include <math.h>
#include <grbl.h>
#include "servo.h"

ADC_HandleTypeDef temp_ADC = {0};
TIM_HandleTypeDef extruderPWM = {0};

static const float R0 = 100e3;
static const float T0 = 25;
static const float beta = 3950;
static const float Rs = 4700;


static const float extruder_thermistor_beta = 3950;

static float extruder0temp = 0;
static float extruder0setpoint = 0;

PIDControllerState extruder0PID = {0};

void tempcontrol_init() {
    // Init controller
    extruder0PID.Kp = 0.066 * 1049;
    extruder0PID.Ti = 15; // Tu/2
    extruder0PID.Td = 3.75; // Tu/8
    extruder0PID.limit = 1049;
    extruder0PID.ts = 1.0f/20.0f;
    resetPID(&extruder0PID);
    
    // Init Analog input
    
    ADC_ChannelConfTypeDef channel_conf;
    
    GPIO_InitTypeDef GPIO_Init = { 0 };

    GPIO_Init.Pin = GPIO_PIN_6;
    GPIO_Init.Mode = GPIO_MODE_ANALOG;
    GPIO_Init.Speed = GPIO_SPEED_LOW;
    GPIO_Init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);

    
    
    temp_ADC.Instance = ADC1;
    
    // enable clock
    __ADC1_CLK_ENABLE();
    
    temp_ADC.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
    temp_ADC.Init.Resolution = ADC_RESOLUTION12b;
    temp_ADC.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    temp_ADC.Init.ScanConvMode = ENABLE;
    temp_ADC.Init.EOCSelection = EOC_SEQ_CONV;
    temp_ADC.Init.ContinuousConvMode = DISABLE;
    temp_ADC.Init.DMAContinuousRequests = DISABLE;
    temp_ADC.Init.NbrOfConversion = 1;
    temp_ADC.Init.DiscontinuousConvMode = DISABLE;
    temp_ADC.Init.NbrOfDiscConversion = 1;
    temp_ADC.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    temp_ADC.Init.ExternalTrigConv = 0;
    
    if (HAL_ADC_Init(&temp_ADC) != HAL_OK) Error_Handler();
    
    channel_conf.Channel = ADC_CHANNEL_6;
    channel_conf.Rank = 1;
    channel_conf.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    channel_conf.Offset = 0;
    
    if (HAL_ADC_ConfigChannel(&temp_ADC, &channel_conf) != HAL_OK) Error_Handler();
    
    if (HAL_ADC_Start(&temp_ADC) != HAL_OK) Error_Handler();
    
    // Init PWM output



    // Extruder PWM pin: PB8
    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    GPIO_Init.Speed = GPIO_SPEED_HIGH;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Alternate = GPIO_AF3_TIM10;

    GPIO_Init.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &GPIO_Init);

    __TIM10_CLK_ENABLE();
    extruderPWM.Instance = TIM10;
    extruderPWM.Init.Period = 4200; //1kHz from 84MHz clock
    extruderPWM.Init.Prescaler = 20;
    extruderPWM.Init.ClockDivision = 0;
    extruderPWM.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&extruderPWM);

    TIM_OC_InitTypeDef ocConfig;


    ocConfig.OCMode = TIM_OCMODE_PWM1;
    ocConfig.Pulse=0;
    ocConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    ocConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    ocConfig.OCFastMode = TIM_OCFAST_DISABLE;
    ocConfig.OCIdleState =TIM_OCIDLESTATE_RESET;
    ocConfig.OCNIdleState = TIM_OCIDLESTATE_RESET;


    HAL_TIM_PWM_ConfigChannel(&extruderPWM, &ocConfig, TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(&extruderPWM, TIM_CHANNEL_1);

    HAL_TIM_Base_Start(&extruderPWM);

}




float tempcontrol_getextrudertemp() {
    return extruder0temp;
}

void tempcontrol_setextrudersetpoint(float t) {
    
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Ensure temperature is set when specified in program.     
    extruder0setpoint = t;
}

void tempcontrol_update(void) {
    float v, R, T, output, e;
    if (HAL_ADC_PollForConversion(&temp_ADC, 0) == HAL_OK) {
        v = HAL_ADC_GetValue(&temp_ADC) / 4095.0f;
        if (v==0) v=1e-4f;
        
        R = Rs / (1-v) - Rs;

        T = 1/(log(R/R0)/beta + (1/(T0+273.15)))-273.15f;
        
        extruder0temp = T;
    
        if (extruder0setpoint <= 0) {
            resetPID(&extruder0PID);
            output = 0;
        } else {
            e = extruder0setpoint - extruder0temp;
            output = computePID(e, &extruder0PID);
        }       
        
        extruderPWM.Instance->CCR1 = max(0, min(output, 1049)); // limit to 1/4 duty cycle
        
        
        
        
    } else {
        extruder0temp = 0.0f;
        tempcontrol_off();
    }
    
    if (HAL_ADC_Start(&temp_ADC) != HAL_OK) Error_Handler();
    



    
}


void tempcontrol_off(void) {
    extruderPWM.Instance->CCR1 = 0;    
}


