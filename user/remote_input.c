#include "remote_input.h"
#include "adc.h"

#define INPUT_SAMPLE_MS 10U
#define KEY_DEBOUNCE_MS 20U
#define ADC_TIMEOUT_MS  2U
#define AXIS_DEADZONE   120

volatile RemoteInputDebug_t g_remote_input;

/* First measured neutral values: LX, LY, RX, RY. Recheck after release. */
static const int32_t s_axis_center[REMOTE_AXIS_COUNT] = {
    2288, 2216, 2027, 1986
};

/* Physical convention: right/up positive, left/down negative. */
static const int8_t s_axis_sign[REMOTE_AXIS_COUNT] = {
    -1, -1, -1, -1
};

static uint32_t s_sample_tick;
static uint32_t s_key_change_tick[REMOTE_KEY_COUNT];
static uint8_t s_candidate_keys;
static uint8_t s_stable_keys;
static uint8_t s_sw12_candidate,s_sw12_stable;
static uint32_t s_sw12_change_tick;

static const uint32_t s_adc_channels[REMOTE_AXIS_COUNT] =
{
    ADC_CHANNEL_3, /* 左摇杆 A：PA3 */
    ADC_CHANNEL_4, /* 左摇杆 B：PA4 */
    ADC_CHANNEL_1, /* 右摇杆 A：PA1 */
    ADC_CHANNEL_2  /* 右摇杆 B：PA2 */
};

static uint8_t Remote_ReadKeys(void)
{
    GPIO_TypeDef * const ports[REMOTE_KEY_COUNT] =
    {
        KEY_TURN_L_GPIO_Port, KEY_TURN_R_GPIO_Port,
        KEY_GRIP1_GPIO_Port, KEY_GRIP2_GPIO_Port,
        KEY_SPEED_GPIO_Port, KEY_ESTOP_GPIO_Port,
        KEY_ENABLE_GPIO_Port, KEY_ROTATE_B_GPIO_Port
    };
    const uint16_t pins[REMOTE_KEY_COUNT] =
    {
        KEY_TURN_L_Pin, KEY_TURN_R_Pin,
        KEY_GRIP1_Pin, KEY_GRIP2_Pin,
        KEY_SPEED_Pin, KEY_ESTOP_Pin,
        KEY_ENABLE_Pin, KEY_ROTATE_B_Pin
    };
    uint8_t mask = 0U;

    for (uint32_t i = 0U; i < REMOTE_KEY_COUNT; ++i)
    {
        if (HAL_GPIO_ReadPin(ports[i], pins[i]) == GPIO_PIN_RESET)
        {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

static uint8_t Remote_ReadSw12(void)
{
    return HAL_GPIO_ReadPin(KEY_SPARE_SW12_GPIO_Port,KEY_SPARE_SW12_Pin)
           ==GPIO_PIN_RESET;
}

static void Remote_UpdateSw12(uint32_t now)
{
    uint8_t raw=Remote_ReadSw12();
    g_remote_input.sw12_pressed=0;
    g_remote_input.sw12_released=0;
    if(raw!=s_sw12_candidate) {
        s_sw12_candidate=raw; s_sw12_change_tick=now;
    }
    if(s_sw12_candidate!=s_sw12_stable &&
       (uint32_t)(now-s_sw12_change_tick)>=KEY_DEBOUNCE_MS) {
        s_sw12_stable=s_sw12_candidate;
        if(s_sw12_stable) {
            g_remote_input.sw12_pressed=1;
            g_remote_input.sw12_press_count++;
        } else g_remote_input.sw12_released=1;
    }
    g_remote_input.sw12_down=s_sw12_stable;
}

static void Remote_UpdateKeys(uint32_t now)
{
    uint8_t raw = Remote_ReadKeys();

    /* Emergency raw input does not wait for the debounce interval. */
    g_remote_input.estop_raw = (raw & REMOTE_KEY_ESTOP) ? 1U : 0U;
    g_remote_input.pressed_edges = 0U;
    g_remote_input.released_edges = 0U;

    for (uint32_t i = 0U; i < REMOTE_KEY_COUNT; ++i)
    {
        uint8_t bit = (uint8_t)(1U << i);

        if ((raw & bit) != (s_candidate_keys & bit))
        {
            s_candidate_keys ^= bit;
            s_key_change_tick[i] = now;
        }

        if ((s_candidate_keys & bit) != (s_stable_keys & bit) &&
            (uint32_t)(now - s_key_change_tick[i]) >= KEY_DEBOUNCE_MS)
        {
            s_stable_keys ^= bit;
            if (s_stable_keys & bit)
            {
                g_remote_input.pressed_edges |= bit;
                g_remote_input.key_press_count[i]++;
            }
            else
            {
                g_remote_input.released_edges |= bit;
            }
        }
    }
    g_remote_input.keys_down = s_stable_keys;
    Remote_UpdateSw12(now);
}

static HAL_StatusTypeDef Remote_ReadAdc(uint32_t channel,
                                        uint16_t *value)
{
    ADC_ChannelConfTypeDef config = {0};
    config.Channel = channel;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    HAL_StatusTypeDef status = HAL_ADC_ConfigChannel(&hadc1, &config);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_ADC_Start(&hadc1);
    if (status == HAL_OK)
    {
        status = HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS);
    }
    if (status == HAL_OK)
    {
        *value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }

    HAL_StatusTypeDef stop_status = HAL_ADC_Stop(&hadc1);
    return (status == HAL_OK) ? stop_status : status;
}

static int16_t Remote_Normalize(uint16_t raw, uint32_t index)
{
    if (index >= REMOTE_AXIS_COUNT) return 0;

    int32_t center = s_axis_center[index];
    int32_t delta = (int32_t)raw - center;
    int32_t result = 0;

    if (delta > AXIS_DEADZONE)
    {
        int32_t span = 4095 - center - AXIS_DEADZONE;
        if (span <= 0) return 0;
        result = (delta - AXIS_DEADZONE) * 1000 / span;
    }
    else if (delta < -AXIS_DEADZONE)
    {
        int32_t span = center - AXIS_DEADZONE;
        if (span <= 0) return 0;
        result = (delta + AXIS_DEADZONE) * 1000 / span;
    }

    result *= s_axis_sign[index];
    if (result > 1000) result = 1000;
    if (result < -1000) result = -1000;
    return (int16_t)result;
}

void RemoteInput_Init(void)
{
    uint32_t now = HAL_GetTick();
    g_remote_input = (RemoteInputDebug_t){0};
    s_sample_tick = now;

    /* A key held at startup does not create a new press event. */
    s_candidate_keys = Remote_ReadKeys();
    s_stable_keys = s_candidate_keys;
    g_remote_input.keys_down = s_stable_keys;
    g_remote_input.estop_raw =
        (s_stable_keys & REMOTE_KEY_ESTOP) ? 1U : 0U;
    for (uint32_t i = 0U; i < REMOTE_KEY_COUNT; ++i)
    {
        s_key_change_tick[i] = now;
    }

    /* A held SW12 at boot does not create a press event. */
    s_sw12_candidate=s_sw12_stable=Remote_ReadSw12();
    s_sw12_change_tick=now;
    g_remote_input.sw12_down=s_sw12_stable;

    /* Hardware ADC calibration, not joystick neutral calibration. */
    g_remote_input.calibration_status = HAL_ADCEx_Calibration_Start(&hadc1);
    g_remote_input.adc_status = g_remote_input.calibration_status;
    if (g_remote_input.calibration_status != HAL_OK)
    {
        g_remote_input.error_count++;
    }
}

void RemoteInput_Process(uint32_t now)
{
    Remote_UpdateKeys(now);

    if (g_remote_input.calibration_status != HAL_OK ||
        (uint32_t)(now - s_sample_tick) < INPUT_SAMPLE_MS)
    {
        return;
    }
    s_sample_tick = now;

    uint16_t sample[REMOTE_AXIS_COUNT];
    for (uint32_t i = 0U; i < REMOTE_AXIS_COUNT; ++i)
    {
        HAL_StatusTypeDef status = Remote_ReadAdc(s_adc_channels[i], &sample[i]);
        g_remote_input.adc_status = status;
        if (status != HAL_OK)
        {
            /* Discard the entire failed frame; no mixed old/new axes. */
            g_remote_input.valid = 0U;
            g_remote_input.error_count++;
            for (uint32_t j = 0U; j < REMOTE_AXIS_COUNT; ++j)
            {
                g_remote_input.axis[j] = 0;
            }
            return;
        }
    }

    for (uint32_t i = 0U; i < REMOTE_AXIS_COUNT; ++i)
    {
        g_remote_input.raw[i] = sample[i];
        g_remote_input.axis[i] = Remote_Normalize(sample[i], i);
    }
    g_remote_input.sample_ms = HAL_GetTick();
    g_remote_input.sample_count++;
    g_remote_input.valid = 1U;
}