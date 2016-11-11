/*

 */

#include <stm32f4xx_hal.h>
#include <grbl.h>
#include "pathplanner.h"
#include "servo.h"

void pathplanner_execute(); //TODO

volatile unsigned long ticks = 0;

void Error_Handler                (void) { for(;;); }
void UsageFault_Handler(void) { for(;;); }
void BusFault_Handler(void) { for(;;); }
void MemMang_Handler(void) { for(;;); }
void HardFault_Handler(void) { for(;;); }

void SysTick_Handler(void) {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}






void io_init() {
    GPIO_InitTypeDef GPIO_Init;

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

    // LED pin
    GPIO_Init.Pin = GPIO_PIN_12;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Speed = GPIO_SPEED_LOW;
    GPIO_Init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_Init);

    //User pin
    GPIO_Init.Pin = GPIO_PIN_0;
    GPIO_Init.Mode = GPIO_MODE_INPUT;
    GPIO_Init.Speed = GPIO_SPEED_LOW;
    GPIO_Init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);





}


int main(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /* Enable HSE oscillator and configure the PLL to 168MHz at 8MHz crystal
     * USB OTG clock = 48MHz
     * APB1 clock = 42MHz
     * APB2 clock = 84MHz
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

    SystemCoreClockUpdate();

    HAL_Init();





    io_init();
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


    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();

	return 0;
}


