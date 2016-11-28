#include <stm32f4xx_hal.h>
#include <math.h>
#include <grbl.h>

#include "pathplanner.h"

TIM_HandleTypeDef motorPWM = {0};

typedef struct  {
    float Kp;
    float Ti;
    float Td;
    float i;
    float d;
    float limit;
    float previous;


} PIDControllerState;



typedef struct {
    PIDControllerState PID;
    TIM_HandleTypeDef encoder;
    int position;
} ServoAxis;


ServoAxis axis[3] = {0};

/*
 * Encoders
 */
void encoders_init(void) {
    GPIO_InitTypeDef GPIO_Init = {0};
    TIM_Encoder_InitTypeDef Encoder_Init = {0};

    __TIM3_CLK_ENABLE();
    __TIM4_CLK_ENABLE();
    __TIM5_CLK_ENABLE();
    
    /* Encoder interface */

    /* common settings */
    Encoder_Init.EncoderMode = TIM_ENCODERMODE_TI12;
    Encoder_Init.IC1Polarity = TIM_ICPOLARITY_RISING;
    Encoder_Init.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_Init.IC1Prescaler = 0;
    Encoder_Init.IC1Filter = 5;
    Encoder_Init.IC2Polarity = TIM_ICPOLARITY_RISING;
    Encoder_Init.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_Init.IC2Prescaler = 0;
    Encoder_Init.IC2Filter = 5;


    /* encoder x on PB4 / PB5, TIM3 */
    GPIO_Init.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_Init.Mode = GPIO_MODE_AF_OD;
    GPIO_Init.Speed = GPIO_SPEED_HIGH;
    GPIO_Init.Pull = GPIO_PULLDOWN;
    GPIO_Init.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_Init);

    axis[0].encoder.Instance = TIM3;
    axis[0].encoder.Init.Period = 65535;
    axis[0].encoder.Init.Prescaler = 0;
    axis[0].encoder.Init.ClockDivision = 0;
    axis[0].encoder.Init.CounterMode = TIM_COUNTERMODE_UP;

    HAL_TIM_Base_Init(&axis[0].encoder);

    HAL_TIM_Encoder_Init(&axis[0].encoder, &Encoder_Init);

    HAL_TIM_Encoder_Start(&axis[0].encoder,TIM_CHANNEL_1|TIM_CHANNEL_2); // All channels

    /* Encoder interface */
    /* encoder y on PB6 / PB7, TIM4 */
    GPIO_Init.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_Init.Mode = GPIO_MODE_AF_OD;
    GPIO_Init.Speed = GPIO_SPEED_HIGH;
    GPIO_Init.Pull = GPIO_PULLDOWN;
    GPIO_Init.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_Init);

    axis[1].encoder.Instance = TIM4;
    axis[1].encoder.Init.Period = 65535;
    axis[1].encoder.Init.Prescaler = 0;
    axis[1].encoder.Init.ClockDivision = 0;
    axis[1].encoder.Init.CounterMode = TIM_COUNTERMODE_UP;

    HAL_TIM_Base_Init(&axis[1].encoder);

    HAL_TIM_Encoder_Init(&axis[1].encoder, &Encoder_Init);

    HAL_TIM_Encoder_Start(&axis[1].encoder,TIM_CHANNEL_1|TIM_CHANNEL_2); // All channels

    /* Encoder interface */
    /* encoder z on PA0 / PA1, TIM5 */
    GPIO_Init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_Init.Mode = GPIO_MODE_AF_OD;
    GPIO_Init.Speed = GPIO_SPEED_HIGH;
    GPIO_Init.Pull = GPIO_PULLDOWN;
    GPIO_Init.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);

    axis[2].encoder.Instance = TIM5;
    axis[2].encoder.Init.Period = 65535;
    axis[2].encoder.Init.Prescaler = 0;
    axis[2].encoder.Init.ClockDivision = 0;
    axis[2].encoder.Init.CounterMode = TIM_COUNTERMODE_UP;

    HAL_TIM_Base_Init(&axis[2].encoder);

    HAL_TIM_Encoder_Init(&axis[2].encoder, &Encoder_Init);

    HAL_TIM_Encoder_Start(&axis[2].encoder,TIM_CHANNEL_1|TIM_CHANNEL_2); // All channels
    

}

static void encoder_read(ServoAxis *axis) {
    short prev, current, delta;
    prev = axis->position;
    current =  axis->encoder.Instance->CNT;
    delta = current - prev;
    axis->position += delta;

}

void encoder_home(uint8_t i) {
    ServoAxis *ax;
    if (i<=2) {
        ax = &axis[i];
        ax->position = 0;
        ax->encoder.Instance->CNT=0;
    }

}

/*
 * Motor control
 */

void motors_init(void) {
    GPIO_InitTypeDef GPIO_Init = {0};

    // Motor enable pin
    GPIO_Init.Pin = GPIO_PIN_8;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Speed = GPIO_SPEED_LOW;
    GPIO_Init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_Init);



    // Motor PWM pins: PA8, PA9, PA10, PB13, PB14, PB15
    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    GPIO_Init.Speed = GPIO_SPEED_HIGH;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Alternate = GPIO_AF1_TIM1;

    GPIO_Init.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);

    GPIO_Init.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &GPIO_Init);




    __TIM1_CLK_ENABLE();
    motorPWM.Instance = TIM1;
    motorPWM.Init.Period = 4200; //20kHz from 84MHz clock
    motorPWM.Init.Prescaler = 0;
    motorPWM.Init.ClockDivision = 0;
    motorPWM.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&motorPWM);

    TIM_OC_InitTypeDef ocConfig;


    ocConfig.OCMode = TIM_OCMODE_PWM1;
    ocConfig.Pulse=0;
    ocConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    ocConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    ocConfig.OCFastMode = TIM_OCFAST_DISABLE;
    ocConfig.OCIdleState =TIM_OCIDLESTATE_RESET;
    ocConfig.OCNIdleState = TIM_OCIDLESTATE_RESET;


    HAL_TIM_PWM_ConfigChannel(&motorPWM, &ocConfig, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&motorPWM, &ocConfig, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&motorPWM, &ocConfig, TIM_CHANNEL_3);

    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0);

    // Enable Timer1 Interrupt
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    HAL_TIM_PWM_Start(&motorPWM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&motorPWM, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&motorPWM, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(&motorPWM);

    // Enable motors
    GPIOC->BSRRL = GPIO_PIN_8; // set pin

}

void motors_setPWM(int p1, int p2, int p3) {
    unsigned short CCER;

    // define polarity for all outputs
    CCER = 0;

    // For each PWM output, activate either the positive or the negative output
    if (p1<0) {
        CCER |= TIM_CCER_CC1NE;
        p1 = -p1;
    } else {
        CCER |= TIM_CCER_CC1E;
    }

    if (p2<0) {
        CCER |= TIM_CCER_CC2NE;
        p2 = -p2;
    } else {
        CCER |= TIM_CCER_CC2E;
    }

    if (p3<0) {
        CCER |= TIM_CCER_CC3NE;
        p3 = -p3;
    } else {
        CCER |= TIM_CCER_CC3E;
    }

    motorPWM.Instance->CCR1 = min(p1, 4199);
    motorPWM.Instance->CCR2 = min(p2, 4199);
    motorPWM.Instance->CCR3 = min(p3, 2099); // 12 V motor for Z
    motorPWM.Instance->CCER = CCER;

}



/*
 * Servo / general
 */

void servo_init() {

    axis[0].PID.Kp = 25;
    axis[0].PID.Ti = 0.05;
    axis[0].PID.Td = 0.01;
    axis[0].PID.limit = 4199;

    axis[1].PID.Kp = 25;
    axis[1].PID.Ti = 0.05;
    axis[1].PID.Td = 0.01;
    axis[1].PID.limit = 4199;

    axis[2].PID.Kp = 50;
    axis[2].PID.Ti = 0.1;
    axis[2].PID.Td = 0.005;
    axis[2].PID.limit = 2099;
    
    motors_init();
    encoders_init();
}

static const float ts = 1.0f/20000.0f;

static float computePID(float input, PIDControllerState *state) {
    float ilimit = state->limit / state->Kp * state->Ti;

    if (isnan(state->previous)) state->previous = input;

    state->i += input * ts;

    state->d = state->d*0.99f + 0.01f*((input-state->previous)/ts);

    if (state->i > ilimit) state->i = ilimit;
    if (state->i < -ilimit) state->i = -ilimit;


    state->previous = input;
    return state->Kp * (input + state->i / state->Ti + state->d * state->Td);
}

void resetPID(PIDControllerState *state) {
    state->i = 0;
    state->d = 0;
    state->previous = NAN;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&motorPWM);

}


int readPosX() {
    encoder_read(&axis[0]);
    return axis[0].position;
}

int readPosY() {
    encoder_read(&axis[1]);
    return axis[1].position;
}


int readPosZ() {
    encoder_read(&axis[2]);
    return axis[2].position;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

    pathplanner_execute();

    encoder_read(&axis[0]);
    encoder_read(&axis[1]);
    encoder_read(&axis[2]);
    
    //do here: if probing active and probe active, copy axisn.position to probe position
    // see grbl probe_state_monitor


    if (sys_probe_state == PROBE_ACTIVE) {
      if (probe_get_state()) {
        sys_probe_state = PROBE_OFF;
        sys.probe_position[0] = axis[0].position;
        sys.probe_position[1] = axis[1].position;
        sys.probe_position[2] = axis[2].position;

        bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
      }
    }



    motors_setPWM(
                computePID(sys.position[0] - axis[0].position, &(axis[0].PID)),
                computePID(sys.position[1] - axis[1].position, &(axis[1].PID)),
                computePID(sys.position[2] - axis[2].position, &(axis[2].PID))
                );


    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);


}





