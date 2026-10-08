#ifndef REMOTE_INPUT_H
#define REMOTE_INPUT_H

#include "main.h"

enum
{
    REMOTE_AXIS_L_A = 0,
    REMOTE_AXIS_L_B,
    REMOTE_AXIS_R_A,
    REMOTE_AXIS_R_B,
    REMOTE_AXIS_COUNT
};

enum
{
    REMOTE_KEY_INDEX_TURN_L = 0,
    REMOTE_KEY_INDEX_TURN_R,
    REMOTE_KEY_INDEX_GRIP1,
    REMOTE_KEY_INDEX_GRIP2,
    REMOTE_KEY_INDEX_SPEED,
    REMOTE_KEY_INDEX_ESTOP,
    REMOTE_KEY_INDEX_ENABLE,
    REMOTE_KEY_COUNT
};

#define REMOTE_KEY_TURN_L (1U << REMOTE_KEY_INDEX_TURN_L)
#define REMOTE_KEY_TURN_R (1U << REMOTE_KEY_INDEX_TURN_R)
#define REMOTE_KEY_GRIP1  (1U << REMOTE_KEY_INDEX_GRIP1)
#define REMOTE_KEY_GRIP2  (1U << REMOTE_KEY_INDEX_GRIP2)
#define REMOTE_KEY_SPEED  (1U << REMOTE_KEY_INDEX_SPEED)
#define REMOTE_KEY_ESTOP  (1U << REMOTE_KEY_INDEX_ESTOP)
#define REMOTE_KEY_ENABLE (1U << REMOTE_KEY_INDEX_ENABLE)

typedef struct
{
    uint16_t raw[REMOTE_AXIS_COUNT];
    int16_t axis[REMOTE_AXIS_COUNT];
    uint8_t valid;
    uint8_t keys_down;
    uint8_t pressed_edges;
    uint8_t released_edges;
    uint8_t estop_raw;
    uint32_t sample_count;
    uint32_t error_count;
    uint32_t sample_ms;
    uint32_t key_press_count[REMOTE_KEY_COUNT];
    HAL_StatusTypeDef calibration_status;
    HAL_StatusTypeDef adc_status;
} RemoteInputDebug_t;

extern volatile RemoteInputDebug_t g_remote_input;

void RemoteInput_Init(void);
void RemoteInput_Process(uint32_t now);

#endif