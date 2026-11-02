#ifndef REMOTE_COMMAND_H
#define REMOTE_COMMAND_H

#include "remote_input.h"

enum {
    REMOTE_SERVO_A = 0,
    REMOTE_SERVO_B,
    REMOTE_SERVO_C1,
    REMOTE_SERVO_C2,
    REMOTE_SERVO_COUNT
};

/* Provisional mapping: check X/Y and polarity on the physical remote. */
#define REMOTE_RIGHT_X_AXIS REMOTE_AXIS_R_A
#define REMOTE_RIGHT_Y_AXIS REMOTE_AXIS_R_B
#define REMOTE_RIGHT_X_SIGN 1
#define REMOTE_RIGHT_Y_SIGN 1

typedef struct {
    float servo_target_deg[REMOTE_SERVO_COUNT];
    int16_t right_x; /* Normalized convention: left negative, right positive. */
    int16_t right_y;
    uint8_t rotate_b;
    uint8_t grip_c1;
    uint8_t grip_c2;
    uint32_t arm_token;
    uint8_t gear; /* 0/1/2 => 30/60/100 percent */
    uint8_t enabled;
    uint8_t estop_latched;
    uint8_t input_fresh;
    uint8_t pressed_edges;
    uint32_t input_fault_count;
    uint32_t sample_update_count;
    uint8_t arm_auto_active,arm_preset_64;
} RemoteCommandDebug_t;

extern volatile RemoteCommandDebug_t g_remote_command;

void RemoteCommand_Init(void);
void RemoteCommand_Process(uint32_t now);

#endif