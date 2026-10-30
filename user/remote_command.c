#include "remote_command.h"

#define WRIST_SPEED_DPS 60.0f
#define WRIST_MIN_DEG 30.0f
#define WRIST_MAX_DEG 150.0f
#define GRIP_CLOSED_DEG 70.0f
#define INPUT_MAX_AGE_MS 100U
#define CENTER_LIMIT 50
#define ACTION_KEYS (REMOTE_KEY_TURN_L | REMOTE_KEY_TURN_R | \
                     REMOTE_KEY_GRIP1 | REMOTE_KEY_GRIP2 | \
                     REMOTE_KEY_SPEED | REMOTE_KEY_ROTATE_B)

volatile RemoteCommandDebug_t g_remote_command;

static uint32_t s_last_sample_count;
static uint32_t s_last_sample_ms;

static float LimitAngle(float angle)
{
    if (angle < WRIST_MIN_DEG) return WRIST_MIN_DEG;
    if (angle > WRIST_MAX_DEG) return WRIST_MAX_DEG;
    return angle;
}

static uint8_t InputsCentered(void)
{
    for (uint32_t i = 0U; i < REMOTE_AXIS_COUNT; ++i) {
        int16_t axis = g_remote_input.axis[i];
        if (axis < -CENTER_LIMIT || axis > CENTER_LIMIT) return 0U;
    }
    return 1U;
}

void RemoteCommand_Init(void)
{
    g_remote_command = (RemoteCommandDebug_t){0};
    g_remote_command.servo_target_deg[REMOTE_SERVO_A] = 90.0f;
    /* Starts disabled; a centered-input ENABLE press starts the bench test. */
    s_last_sample_count = g_remote_input.sample_count;
    s_last_sample_ms = g_remote_input.sample_ms;
}

void RemoteCommand_Process(uint32_t now)
{
    uint8_t pressed = g_remote_input.pressed_edges;
    uint8_t fresh = (g_remote_input.valid &&
                     g_remote_input.sample_count != 0U &&
                     (uint32_t)(now - g_remote_input.sample_ms) <=
                         INPUT_MAX_AGE_MS) ? 1U : 0U;
    uint8_t new_sample =
        (g_remote_input.sample_count != s_last_sample_count) ? 1U : 0U;
    uint32_t dt_ms = 0U;

    if (new_sample) {
        dt_ms = g_remote_input.sample_ms - s_last_sample_ms;
        s_last_sample_count = g_remote_input.sample_count;
        s_last_sample_ms = g_remote_input.sample_ms;
    }

    g_remote_command.input_fresh = fresh;
    g_remote_command.pressed_edges = pressed;
    g_remote_command.right_x = (int16_t)(
        g_remote_input.axis[REMOTE_RIGHT_X_AXIS] * REMOTE_RIGHT_X_SIGN);
    g_remote_command.right_y = (int16_t)(
        g_remote_input.axis[REMOTE_RIGHT_Y_AXIS] * REMOTE_RIGHT_Y_SIGN);

    /* Raw emergency input wins, including simultaneous ENABLE presses. */
    if (g_remote_input.estop_raw ||
        (g_remote_input.keys_down & REMOTE_KEY_ESTOP)) {
        g_remote_command.estop_latched = 1U;
        g_remote_command.enabled = 0U;
        return;
    }

    if (!fresh) {
        if (g_remote_command.enabled) g_remote_command.input_fault_count++;
        g_remote_command.enabled = 0U;
        return;
    }

    if ((pressed & REMOTE_KEY_ENABLE) && InputsCentered() &&
        (g_remote_input.keys_down & ACTION_KEYS) == 0U) {
        g_remote_command.estop_latched = 0U;
        g_remote_command.enabled = 1U;
        dt_ms = 0U; /* Do not integrate time spent disabled. */
    }
    if (!g_remote_command.enabled || g_remote_command.estop_latched) return;

    if (pressed & REMOTE_KEY_ROTATE_B) g_remote_command.rotate_b ^= 1U;
    if (pressed & REMOTE_KEY_GRIP1) g_remote_command.grip_c1 ^= 1U;
    if (pressed & REMOTE_KEY_GRIP2) g_remote_command.grip_c2 ^= 1U;

    if (new_sample && dt_ms > 0U && dt_ms <= INPUT_MAX_AGE_MS) {
        float change = (float)g_remote_command.right_x * 0.001f *
                       WRIST_SPEED_DPS * (float)dt_ms * 0.001f;
        float angle = g_remote_command.servo_target_deg[REMOTE_SERVO_A];
        g_remote_command.servo_target_deg[REMOTE_SERVO_A] =
            LimitAngle(angle - change);
        g_remote_command.sample_update_count++;
    }

    g_remote_command.servo_target_deg[REMOTE_SERVO_B] =
        g_remote_command.rotate_b ? 180.0f : 0.0f;
    g_remote_command.servo_target_deg[REMOTE_SERVO_C1] =
        g_remote_command.grip_c1 ? GRIP_CLOSED_DEG : 0.0f;
    g_remote_command.servo_target_deg[REMOTE_SERVO_C2] =
        g_remote_command.grip_c2 ? GRIP_CLOSED_DEG : 0.0f;
}