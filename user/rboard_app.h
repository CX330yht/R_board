#ifndef RBOARD_APP_H
#define RBOARD_APP_H

#include "main.h"

typedef struct
{
    uint32_t uptime_ms;
    uint32_t loop_count;
    uint32_t uart_tx_count;
    uint32_t uart_error_count;
    uint16_t tim2_counter;
    uint8_t estop_pressed_raw;
    uint8_t enable_pressed_raw;
    uint8_t nrf_irq_level;
    HAL_StatusTypeDef tim2_start_status;
    HAL_StatusTypeDef uart_last_status;
} RBoardDebug_t;

extern volatile RBoardDebug_t g_rboard_debug;

void RBoardApp_Init(void);
void RBoardApp_Process(void);

#endif