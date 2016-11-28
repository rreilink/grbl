/*

 */

#include <stm32f4xx_hal.h>
#include <grbl.h>
#include "pathplanner.h"
#include "servo.h"
#include <math.h>


//void pathplanner_execute(); //TODO

volatile unsigned long ticks = 0;

void Error_Handler                (void) { for(;;); }
void UsageFault_Handler(void) {
    int a;
    a = SCB->CFSR;
    for(;;);
}
void BusFault_Handler(void) {

    int a;
    a = SCB->CFSR;
    for(;;);
}
void MemMang_Handler(void) { for(;;); }
void HardFault_Handler(void) { for(;;); }

void RCC_IRQHandler(void) { for(;;); }


void SysTick_Handler(void) {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}


void io_init() {
    GPIO_InitTypeDef GPIO_Init = { 0 };

    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();
    __GPIOD_CLK_ENABLE();
    __GPIOE_CLK_ENABLE();


    // X,Y,Z home switches + probe input

    GPIO_Init.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_Init.Mode = GPIO_MODE_INPUT;
    GPIO_Init.Speed = GPIO_SPEED_LOW;
    GPIO_Init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_Init);

    // Timing debug pin
    GPIO_Init.Pin = GPIO_PIN_10;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Speed = GPIO_SPEED_HIGH;
    GPIO_Init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_Init);

    //User pin
    GPIO_Init.Pin = GPIO_PIN_0;
    GPIO_Init.Mode = GPIO_MODE_INPUT;
    GPIO_Init.Speed = GPIO_SPEED_LOW;
    GPIO_Init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);

    //Stepper dir & enabled - PC11 & PC13
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Speed = GPIO_SPEED_HIGH;
    GPIO_Init.Pull = GPIO_NOPULL;

    GPIO_Init.Pin = GPIO_PIN_11 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &GPIO_Init);
    bit_false(GPIOC->ODR, GPIO_PIN_13); // set pin 13 (/enable)

    //stepper pulse - PE5
    GPIO_Init.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOE, &GPIO_Init);
}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Enable HSE oscillator and configure the PLL to 84MHz at 8MHz external clock input
   * APB1 clock = 42MHz
   * APB2 clock = 84MHz
   */

  /* Enable HSE Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


int main(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk
    | SCB_SHCSR_MEMFAULTENA_Msk;

    HAL_Init();
    SystemClock_Config();

    io_init();

#if 0
    /* -2- Configure PA05 IO in output push-pull mode to
           drive external LED */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* -3- Toggle PA05 IO in an infinite loop */
    while (1)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

      /* Insert delay 100 ms */
      HAL_Delay(50);
    }
#endif

#if 0    
    encoders_init();
    //motors_init();
    //motors_setPWM(-400,-400,0);

    serial_init();
    for(;;) {
        HAL_Delay(50);
        serial_write('\n');
        print_uint32_base10(readPosX());
        serial_write(',');
        print_uint32_base10(readPosZ());

    }
#endif

    serial_init();
    servo_init();


    settings_restore(0xff);
    settings.flags |= BITFLAG_HOMING_ENABLE;

    plan_reset();
    pathplanner_init();

    // Reset system variables.
    sys.abort = false;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys.suspend = false;
    sys.soft_limit = false;

/*
    bit_true(GPIOC->ODR, GPIO_PIN_13); // set pin 13 (/enable)

    for(;;) {
        bit_true(GPIOE->ODR, GPIO_PIN_5); // set pin 5 (step)
        HAL_Delay (10);
        bit_false(GPIOE->ODR, GPIO_PIN_5); // clear pin 5 (step)
        HAL_Delay (10);
    }
*/
    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();

	return 0;
}



