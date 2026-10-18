
/*
 * STM32F411CEU6 / AMR - USART1 diagnostic firmware
 * Based on pin configuration in AMR_STM32F411 (PA9 TX, PA10 RX).
 *
 * Connect:
 *   Raspberry Pi 4 GPIO14 TX (physical pin 8)  -> STM32 PA10 RX
 *   Raspberry Pi 4 GPIO15 RX (physical pin 10) <- STM32 PA9 TX
 *   Raspberry Pi GND                           <-> STM32 GND
 *
 * USART1: 9600 baud, 8 data bits, no parity, 1 stop bit.
 * Command: T 000001\r   ->   ACK 000001\r\n
 *          p\r          ->   PONG\r\n
 *          q\r          ->   UARTONLY ...\r\n
 *          s\r          ->   OK MOTOR_DISABLED\r\n
 *
 * MOTOR SAFETY: Does not initialize/start TIM1-TIM4 or enable BTS7960.
 * Disconnect motor battery and BTS7960 control wiring while testing.
 * This file is for diagnosis, NOT a robot motor controller.
 */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Required by HAL UART configuration and STM32Cube-generated IRQ source. */
UART_HandleTypeDef huart1;
/* The current stm32f4xx_it.c still references huart2 in USART2_IRQHandler.
 * Keep this unused symbol so the existing project links without edits.
 * USART2 is NOT initialized or enabled in this firmware.
 */
UART_HandleTypeDef huart2;

#define UART_RX_LINE_SIZE       64U
#define UART_TX_REPORT_SIZE   256U
#define UART_RX_TIMEOUT_MS    100U
#define UART_TX_TIMEOUT_MS   1000U

static uint8_t uart_byte;
static char uart_line[UART_RX_LINE_SIZE];
static size_t uart_index;
static uint8_t uart_discard;
static uint32_t uart_rx_count;
static uint32_t uart_rx_errors;
static uint32_t uart_tx_errors;
static uint32_t uart_last_error;
static uint32_t uart_bad_commands;
static uint32_t uart_long_lines;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void UART_Diag_Send(const char *text);
static void UART_Diag_Process(const char *command);
static void UART_Diag_Poll(void);

static void UART_Diag_Send(const char *text)
{
    HAL_StatusTypeDef result = HAL_UART_Transmit(
        &huart1,
        (uint8_t *)text,
        (uint16_t)strlen(text),
        UART_TX_TIMEOUT_MS
    );
    if (result != HAL_OK)
    {
        uart_tx_errors++;
    }
}

static void UART_Diag_Process(const char *command)
{
    char report[UART_TX_REPORT_SIZE];
    size_t n = strlen(command);
    size_t i;

    /* Protocol for pi_uart_seq.py: T 000001\r -> ACK 000001\r\n */
    if (n == 8U && command[0] == 'T' && command[1] == ' ')
    {
        uint8_t valid = 1U;
        for (i = 2U; i < 8U; ++i)
        {
            if (command[i] < '0' || command[i] > '9')
            {
                valid = 0U;
                break;
            }
        }
        if (valid)
        {
            (void)snprintf(report, sizeof(report), "ACK %.6s\r\n", command + 2);
            UART_Diag_Send(report);
            return;
        }
    }

    if (strcmp(command, "p") == 0 || strcmp(command, "PING") == 0)
    {
        UART_Diag_Send("PONG\r\n");
    }
    else if (strcmp(command, "q") == 0)
    {
        (void)snprintf(report, sizeof(report),
            "UARTONLY RX=%lu ERR=%lu LAST=0x%lX TXERR=%lu BAD=%lu LONG=%lu CLK=%lu\r\n",
            (unsigned long)uart_rx_count,
            (unsigned long)uart_rx_errors,
            (unsigned long)uart_last_error,
            (unsigned long)uart_tx_errors,
            (unsigned long)uart_bad_commands,
            (unsigned long)uart_long_lines,
            (unsigned long)HAL_RCC_GetHCLKFreq());
        UART_Diag_Send(report);
    }
    else if (strcmp(command, "e") == 0)
    {
        UART_Diag_Send("0 0\r\n");
    }
    else if (strcmp(command, "s") == 0)
    {
        UART_Diag_Send("OK MOTOR_DISABLED\r\n");
    }
    else
    {
        /* Show malformed bytes exactly, so we can diagnose bit flips. */
        size_t pos;
        uart_bad_commands++;
        pos = (size_t)snprintf(report, sizeof(report), "BADHEX");
        for (i = 0U; i < n && pos + 4U < sizeof(report); ++i)
        {
            pos += (size_t)snprintf(report + pos, sizeof(report) - pos,
                                    " %02X", (unsigned int)(uint8_t)command[i]);
        }
        (void)snprintf(report + pos, sizeof(report) - pos, "\r\n");
        UART_Diag_Send(report);
    }
}

static void UART_Diag_Poll(void)
{
    HAL_StatusTypeDef status = HAL_UART_Receive(
        &huart1, &uart_byte, 1U, UART_RX_TIMEOUT_MS);

    if (status == HAL_TIMEOUT)
        return;

    if (status != HAL_OK)
    {
        uart_last_error = HAL_UART_GetError(&huart1);
        uart_rx_errors++;
        /* On STM32F4, SR read followed by DR read clears ORE/NE/FE. */
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        uart_index = 0U;
        uart_discard = 0U;
        return;
    }

    uart_rx_count++;
    if (uart_byte == '\r' || uart_byte == '\n')
    {
        if (!uart_discard && uart_index > 0U)
        {
            uart_line[uart_index] = '\0';
            UART_Diag_Process(uart_line);
        }
        uart_index = 0U;
        uart_discard = 0U;
    }
    else if (!uart_discard)
    {
        if (uart_index < UART_RX_LINE_SIZE - 1U)
        {
            uart_line[uart_index++] = (char)uart_byte;
        }
        else
        {
            uart_index = 0U;
            uart_discard = 1U;
            uart_long_lines++;
        }
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Keep driver enables low. NO motor / encoder timer initialization. */
    MX_GPIO_Init();
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_12 | GPIO_PIN_13 |
                      GPIO_PIN_14 | GPIO_PIN_15,
                      GPIO_PIN_RESET);

    /* HAL_UART_Init calls HAL_UART_MspInit in stm32f4xx_hal_msp.c,
       which sets PA9 and PA10 to AF7_USART1 and enables USART1 clock. */
    MX_USART1_UART_Init();

    while (1)
    {
        UART_Diag_Poll();
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_12 | GPIO_PIN_13 |
                      GPIO_PIN_14 | GPIO_PIN_15,
                      GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 |
                          GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        /* An error occurred during initialization. */
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
