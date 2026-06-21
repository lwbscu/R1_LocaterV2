#include "driver_encoder.h"
#include "locater_config.h"
#include "tim.h"

static uint32_t s_x_prev_raw;
static uint32_t s_y_prev_raw;
static int32_t s_x_total_count;
static int32_t s_y_total_count;
static volatile bool s_x_index_seen;
static volatile bool s_y_index_seen;
static bool s_x_pulse_seen;
static bool s_y_pulse_seen;

static int32_t encoder_delta_32(uint32_t now, uint32_t previous)
{
    return (int32_t)(now - previous);
}

static int32_t encoder_delta_16(uint32_t now, uint32_t previous)
{
    return (int32_t)(int16_t)((uint16_t)now - (uint16_t)previous);
}

void Driver_Encoder_Init(void)
{
    Driver_Encoder_Reset();

    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
}

void Driver_Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);

    s_x_prev_raw = 0U;
    s_y_prev_raw = 0U;
    s_x_total_count = 0;
    s_y_total_count = 0;
    s_x_index_seen = false;
    s_y_index_seen = false;
    s_x_pulse_seen = false;
    s_y_pulse_seen = false;
}

void Driver_Encoder_GetSnapshot(Encoder_Snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    const uint32_t x_now = __HAL_TIM_GET_COUNTER(&htim2);
    const uint32_t y_now = __HAL_TIM_GET_COUNTER(&htim3);

    int32_t x_delta = encoder_delta_32(x_now, s_x_prev_raw);
    int32_t y_delta = encoder_delta_16(y_now, s_y_prev_raw);

#if LOCATER_ENCODER_X_REVERSED
    x_delta = -x_delta;
#endif
#if LOCATER_ENCODER_Y_REVERSED
    y_delta = -y_delta;
#endif

    s_x_total_count += x_delta;
    s_y_total_count += y_delta;
    if (x_delta != 0) {
        s_x_pulse_seen = true;
    }
    if (y_delta != 0) {
        s_y_pulse_seen = true;
    }
    s_x_prev_raw = x_now;
    s_y_prev_raw = y_now;

    snapshot->x_raw_count = x_now;
    snapshot->y_raw_count = y_now;
    snapshot->x_delta_count = x_delta;
    snapshot->y_delta_count = y_delta;
    snapshot->x_total_count = s_x_total_count;
    snapshot->y_total_count = s_y_total_count;
    snapshot->x_index_seen = s_x_index_seen;
    snapshot->y_index_seen = s_y_index_seen;
    snapshot->x_pulse_seen = s_x_pulse_seen;
    snapshot->y_pulse_seen = s_y_pulse_seen;
}

void Driver_Encoder_ClearIndexFlags(void)
{
    s_x_index_seen = false;
    s_y_index_seen = false;
}

void Driver_Encoder_EXTI_Callback(uint16_t gpio_pin)
{
    if (gpio_pin == ENCODER_X_Z_Pin) {
        s_x_index_seen = true;
    } else if (gpio_pin == ENCODER_Y_Z_Pin) {
        s_y_index_seen = true;
    }
}
