#include <stm32f4xx_hal.h>
#include <grbl.h>

UART_HandleTypeDef UART_stdio;

uint8_t serial_rx_buffer[RX_BUFFER_SIZE];
uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

uint8_t serial_tx_buffer[TX_BUFFER_SIZE];
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;

// Writes one byte to the TX serial buffer. Called by main program.
// TODO: Check if we can speed this up for writing strings, rather than single bytes.
void serial_write(uint8_t data) {
  // Calculate next head
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_BUFFER_SIZE) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == serial_tx_buffer_tail) {
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  // Enable TXE to make sure tx-streaming is running
  __HAL_UART_ENABLE_IT(&UART_stdio, UART_IT_TXE);
}


// Data Register Empty Interrupt handler
void UART_TXE_INT()
{
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)

  {
    // Send a byte from the buffer
    UART_stdio.Instance->DR = serial_tx_buffer[tail];

    // Update tail position
    tail++;
    if (tail == TX_BUFFER_SIZE) { tail = 0; }

    serial_tx_buffer_tail = tail;
  }

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial_tx_buffer_head) { __HAL_UART_DISABLE_IT(&UART_stdio, UART_IT_TXE); }
}

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
  uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail];

    tail++;
    if (tail == RX_BUFFER_SIZE) { tail = 0; }
    serial_rx_buffer_tail = tail;

    return data;
  }
}


void UART_RX_INT()
{
  uint8_t data = UART_stdio.Instance->DR;
  uint8_t next_head;

  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    case CMD_STATUS_REPORT: bit_true_atomic(sys_rt_exec_state, EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   bit_true_atomic(sys_rt_exec_state, EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD); break; // Set as true
    case CMD_SAFETY_DOOR:   bit_true_atomic(sys_rt_exec_state, EXEC_SAFETY_DOOR); break; // Set as true
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    default: // Write character to buffer
      next_head = serial_rx_buffer_head + 1;
      if (next_head == RX_BUFFER_SIZE) { next_head = 0; }

      // Write data to buffer unless it is full.
      if (next_head != serial_rx_buffer_tail) {
        serial_rx_buffer[serial_rx_buffer_head] = data;
        serial_rx_buffer_head = next_head;

      }
      //TODO: else alarm on overflow?
  }
}


void serial_reset_read_buffer()
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}

// Returns the number of bytes used in the RX serial buffer.
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}

void serial_init() {
    GPIO_InitTypeDef GPIO_Init;

    /* GPIOA Configuration:  UART2 TX on PA2, UART2 RX on PA3 */
    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    GPIO_Init.Speed = GPIO_SPEED_HIGH;
    GPIO_Init.Alternate = GPIO_AF7_USART2;

    GPIO_Init.Pin = GPIO_PIN_2;
    GPIO_Init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);

    GPIO_Init.Pin = GPIO_PIN_3;
    GPIO_Init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);

    /* enable peripheral clock for USART2 */
    __USART2_CLK_ENABLE();

    UART_stdio.Instance = USART2;
    UART_stdio.Init.BaudRate = BAUD_RATE;
    UART_stdio.Init.WordLength = UART_WORDLENGTH_8B;
    UART_stdio.Init.StopBits = UART_STOPBITS_1;
    UART_stdio.Init.Parity = UART_PARITY_NONE;
    UART_stdio.Init.Mode = UART_MODE_TX_RX;
    UART_stdio.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UART_stdio.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&UART_stdio);


    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);

    /* Enable Timer1 Interrupt */
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    __HAL_UART_ENABLE_IT(&UART_stdio, UART_IT_RXNE);
}

void USART2_IRQHandler (void) {
    UART_HandleTypeDef *huart = &UART_stdio;
    uint32_t tmp1 = 0, tmp2 = 0;

    tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE);
    tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE);
    /* UART in mode Receiver ---------------------------------------------------*/
    if((tmp1 != RESET) && (tmp2 != RESET))
    {
        UART_RX_INT(huart);
    }

    tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_TXE);
    tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE);
    /* UART in mode Transmitter ------------------------------------------------*/
    if((tmp1 != RESET) && (tmp2 != RESET))
    {
        UART_TXE_INT(huart);
    }

}
