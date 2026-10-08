#include "rboard_app.h"
#include "tim.h"
#include "usart.h"
#include "remote_input.h"
#include "nrf24.h"

volatile RBoardDebug_t g_rboard_debug;

static uint32_t s_led_tick;
static uint32_t s_uart_tick;

static void RBoard_SendAlive(void)
{
    static const uint8_t message[] = "R_BOARD ALIVE\r\n";

    HAL_StatusTypeDef status = HAL_UART_Transmit(
        &huart1, message, (uint16_t)(sizeof(message) - 1U), 10U);

    g_rboard_debug.uart_last_status = status;
    if (status == HAL_OK)
    {
        g_rboard_debug.uart_tx_count++;
    }
    else
    {
        g_rboard_debug.uart_error_count++;
    }
}

void RBoardApp_Init(void)
{
    g_rboard_debug = (RBoardDebug_t){0};

    /* LEDs are active-low. Leave the radio disabled in this stage. */
    HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_AUX_GPIO_Port, LED_AUX_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

    g_rboard_debug.tim2_start_status = HAL_TIM_Base_Start(&htim2);
    RemoteInput_Init();
    (void)NRF24_Init();
    RBoard_SendAlive();

    s_led_tick = HAL_GetTick();
    s_uart_tick = s_led_tick;
}

void RBoardApp_Process(void)
{
    uint32_t now = HAL_GetTick();
    RemoteInput_Process(now);

    g_rboard_debug.uptime_ms = now;
    g_rboard_debug.loop_count++;
    g_rboard_debug.tim2_counter =
        (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

    /* Raw observation only: no debounce or emergency latch yet. */
    g_rboard_debug.estop_pressed_raw =
        (HAL_GPIO_ReadPin(KEY_ESTOP_GPIO_Port, KEY_ESTOP_Pin)
         == GPIO_PIN_RESET) ? 1U : 0U;
    g_rboard_debug.enable_pressed_raw =
        (HAL_GPIO_ReadPin(KEY_ENABLE_GPIO_Port, KEY_ENABLE_Pin)
         == GPIO_PIN_RESET) ? 1U : 0U;
    g_rboard_debug.nrf_irq_level =
        (HAL_GPIO_ReadPin(NRF_IRQ_GPIO_Port, NRF_IRQ_Pin)
         == GPIO_PIN_SET) ? 1U : 0U;

    if ((uint32_t)(now - s_led_tick) >= 500U)
    {
        s_led_tick = now;
        HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
    }

    if ((uint32_t)(now - s_uart_tick) >= 1000U)
    {
        s_uart_tick = now;
        RBoard_SendAlive();
    }

    /* A failed peripheral does not stop the application loop. */
    uint8_t fault =
        (g_rboard_debug.tim2_start_status != HAL_OK ||
         g_rboard_debug.uart_last_status != HAL_OK ||
         g_remote_input.calibration_status != HAL_OK ||
         g_remote_input.adc_status != HAL_OK ||
         g_nrf24_debug.probe_status != HAL_OK) ? 1U : 0U;

    HAL_GPIO_WritePin(LED_AUX_GPIO_Port, LED_AUX_Pin,
                 fault ? GPIO_PIN_RESET : GPIO_PIN_SET);
}