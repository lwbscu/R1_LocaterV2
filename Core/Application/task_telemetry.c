#include "task_locater.h"
#include "FreeRTOS.h"
#include "task.h"
#include "driver_uart.h"
#include "locater_config.h"
#include <stdio.h>
#include <string.h>

typedef void (*TelemetryTransmit_t)(const uint8_t *data, uint16_t len);

static void send_firewater_values(TelemetryTransmit_t transmit, const float *values, uint16_t count);
static void send_locater_csv_v3(TelemetryTransmit_t transmit, const Locater_State_t *state);
static void send_locater_diag_csv(TelemetryTransmit_t transmit, const Locater_State_t *state);
static void send_host_pose_frame(const Locater_State_t *state);
static void get_host_pose_values(const Locater_State_t *state, float values[LOCATER_HOST_FRAME_FLOAT_COUNT]);

static int append_char(char *line, size_t line_size, int offset, char value)
{
    if (offset < 0 || (size_t)offset >= line_size) {
        return offset;
    }

    line[offset++] = value;
    if ((size_t)offset < line_size) {
        line[offset] = '\0';
    }
    return offset;
}

static int append_fixed3(char *line, size_t line_size, int offset, float value)
{
    if (offset < 0 || (size_t)offset >= line_size) {
        return offset;
    }

    int32_t milli = (value >= 0.0f)
                        ? (int32_t)(value * 1000.0f + 0.5f)
                        : (int32_t)(value * 1000.0f - 0.5f);
    const char *sign = "";
    uint32_t mag;

    if (milli < 0) {
        sign = "-";
        mag = (uint32_t)(-milli);
    } else {
        mag = (uint32_t)milli;
    }

    const uint32_t whole = mag / 1000U;
    const uint32_t frac = mag % 1000U;
    const int len = snprintf(line + offset, line_size - (size_t)offset,
                             "%s%lu.%03lu",
                             sign,
                             (unsigned long)whole,
                             (unsigned long)frac);
    if (len < 0) {
        return offset;
    }

    offset += len;
    if ((size_t)offset >= line_size) {
        offset = (int)line_size - 1;
        line[offset] = '\0';
    }
    return offset;
}

static int append_u32(char *line, size_t line_size, int offset, uint32_t value)
{
    if (offset < 0 || (size_t)offset >= line_size) {
        return offset;
    }

    const int len = snprintf(line + offset, line_size - (size_t)offset,
                             "%lu", (unsigned long)value);
    if (len < 0) {
        return offset;
    }

    offset += len;
    if ((size_t)offset >= line_size) {
        offset = (int)line_size - 1;
        line[offset] = '\0';
    }
    return offset;
}

static int append_i32(char *line, size_t line_size, int offset, int32_t value)
{
    if (offset < 0 || (size_t)offset >= line_size) {
        return offset;
    }

    const int len = snprintf(line + offset, line_size - (size_t)offset,
                             "%ld", (long)value);
    if (len < 0) {
        return offset;
    }

    offset += len;
    if ((size_t)offset >= line_size) {
        offset = (int)line_size - 1;
        line[offset] = '\0';
    }
    return offset;
}

static int append_csv_fixed3(char *line, size_t line_size, int offset, float value, bool first)
{
    if (!first) {
        offset = append_char(line, line_size, offset, ',');
    }
    return append_fixed3(line, line_size, offset, value);
}

static int append_csv_u32(char *line, size_t line_size, int offset, uint32_t value, bool first)
{
    if (!first) {
        offset = append_char(line, line_size, offset, ',');
    }
    return append_u32(line, line_size, offset, value);
}

static int append_csv_i32(char *line, size_t line_size, int offset, int32_t value, bool first)
{
    if (!first) {
        offset = append_char(line, line_size, offset, ',');
    }
    return append_i32(line, line_size, offset, value);
}

static uint32_t locater_status_mask(const Locater_State_t *state)
{
    if (state == NULL) {
        return 0U;
    }

    uint32_t mask = 0U;
    const bool encoder_x_valid = state->x_pulse_seen;
    const bool encoder_y_valid = state->y_pulse_seen;

    if (encoder_x_valid && encoder_y_valid) {
        mask |= (1UL << 0); /* encoder_valid */
    }
    if (state->h30_has_attitude) {
        mask |= (1UL << 1); /* h30_valid: attitude/yaw frame parsed */
    }
    if (state->lidar_valid) {
        mask |= (1UL << 2); /* lidar_valid */
    }
    if (state->lidar_online) {
        mask |= (1UL << 3); /* lidar_online */
    }
    if (state->dt35_1_valid) {
        mask |= (1UL << 4); /* dt35_1_valid */
    }
    if (state->dt35_2_valid) {
        mask |= (1UL << 5); /* dt35_2_valid */
    }
    if (state->h30_has_attitude || state->lidar_online) {
        mask |= (1UL << 6); /* fusion_valid */
    }
    if (state->h30_crc_error_count > 0U || state->h30_frame_error_count > 0U) {
        mask |= (1UL << 7); /* h30_error */
    }
    if (state->lidar_checksum_error_count > 0U || state->lidar_frame_error_count > 0U) {
        mask |= (1UL << 8); /* lidar_error */
    }
    if (state->x_index_seen || state->y_index_seen) {
        mask |= (1UL << 9); /* encoder_index_seen */
    }
    if (encoder_x_valid) {
        mask |= (1UL << 10); /* encoder_1_valid: TIM2 orthogonal encoder 1 pulse seen */
    }
    if (encoder_y_valid) {
        mask |= (1UL << 11); /* encoder_2_valid: TIM3 orthogonal encoder 2 pulse seen */
    }

    return mask;
}

static float host_wrap_yaw_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle <= -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static void get_host_start_pose(float *x_cm, float *y_cm, float *yaw_deg)
{
    if (x_cm == NULL || y_cm == NULL || yaw_deg == NULL) {
        return;
    }

#if LOCATER_HOST_START_SIDE == LOCATER_HOST_START_SIDE_BLUE
    *x_cm = LOCATER_HOST_BLUE_START_X_CM;
    *y_cm = LOCATER_HOST_BLUE_START_Y_CM;
    *yaw_deg = LOCATER_HOST_BLUE_START_YAW_DEG;
#else
    *x_cm = LOCATER_HOST_RED_START_X_CM;
    *y_cm = LOCATER_HOST_RED_START_Y_CM;
    *yaw_deg = LOCATER_HOST_RED_START_YAW_DEG;
#endif
}

static void get_host_pose_values(const Locater_State_t *state, float values[LOCATER_HOST_FRAME_FLOAT_COUNT])
{
    if (state == NULL || values == NULL) {
        return;
    }

    float x_cm = state->x_cm;
    float y_cm = state->y_cm;
    float yaw_deg = state->yaw_deg;

#if LOCATER_HOST_OFFSET_LOCAL_ODOM
    if (!state->lidar_valid) {
        float start_x_cm = 0.0f;
        float start_y_cm = 0.0f;
        float start_yaw_deg = 0.0f;

        get_host_start_pose(&start_x_cm, &start_y_cm, &start_yaw_deg);
        x_cm += start_x_cm;
        y_cm += start_y_cm;
        yaw_deg += start_yaw_deg;
    }
#endif

    values[0] = x_cm;
    values[1] = y_cm;
    values[2] = host_wrap_yaw_deg(yaw_deg);
    values[3] = state->lidar_x_cm;
    values[4] = state->lidar_y_cm;
    values[5] = state->lidar_yaw_deg;
    values[6] = state->encoder_x_cm;
    values[7] = state->encoder_y_cm;
    values[8] = state->h30_yaw_deg;
    values[9] = state->dt35_1_mm;
    values[10] = state->dt35_2_mm;
}

static void send_firewater_values(TelemetryTransmit_t transmit, const float *values, uint16_t count)
{
    if (transmit == NULL || values == NULL || count == 0U) {
        return;
    }

    char line[192];
    int len = 0;

    for (uint16_t i = 0U; i < count; i++) {
        if (i > 0U) {
            len = append_char(line, sizeof(line), len, ',');
        }
        len = append_fixed3(line, sizeof(line), len, values[i]);
    }

    len = append_char(line, sizeof(line), len, '\r');
    len = append_char(line, sizeof(line), len, '\n');

    if (len > 0) {
        transmit((const uint8_t *)line, (uint16_t)len);
    }
}

static void send_locater_csv_v3(TelemetryTransmit_t transmit, const Locater_State_t *state)
{
    if (transmit == NULL || state == NULL) {
        return;
    }

    char line[192];
    int len = 0;
    bool first = true;
    const uint32_t status = locater_status_mask(state);

#define APPEND_F3(value_)                         \
    do {                                          \
        len = append_csv_fixed3(line, sizeof(line), len, (value_), first); \
        first = false;                            \
    } while (0)

#define APPEND_U32(value_)                        \
    do {                                          \
        len = append_csv_u32(line, sizeof(line), len, (uint32_t)(value_), first); \
        first = false;                            \
    } while (0)

    APPEND_F3(state->x_cm);
    APPEND_F3(state->y_cm);
    APPEND_F3(state->yaw_deg);
    APPEND_F3(state->lidar_x_cm);
    APPEND_F3(state->lidar_y_cm);
    APPEND_F3(state->lidar_yaw_deg);
    APPEND_F3(state->encoder_x_cm);
    APPEND_F3(state->encoder_y_cm);
    APPEND_F3(state->h30_yaw_deg);
    APPEND_F3(state->dt35_1_mm);
    APPEND_F3(state->dt35_2_mm);
    APPEND_U32(status);

#undef APPEND_F3
#undef APPEND_U32

    len = append_char(line, sizeof(line), len, '\r');
    len = append_char(line, sizeof(line), len, '\n');

    if (len > 0) {
        transmit((const uint8_t *)line, (uint16_t)len);
    }
}

static void send_locater_diag_csv(TelemetryTransmit_t transmit, const Locater_State_t *state)
{
    if (transmit == NULL || state == NULL) {
        return;
    }

    char line[512];
    int len = 0;
    bool first = true;
    const uint32_t status = locater_status_mask(state);

#define APPEND_F3(value_)                         \
    do {                                          \
        len = append_csv_fixed3(line, sizeof(line), len, (value_), first); \
        first = false;                            \
    } while (0)

#define APPEND_U32(value_)                        \
    do {                                          \
        len = append_csv_u32(line, sizeof(line), len, (uint32_t)(value_), first); \
        first = false;                            \
    } while (0)

#define APPEND_I32(value_)                        \
    do {                                          \
        len = append_csv_i32(line, sizeof(line), len, (int32_t)(value_), first); \
        first = false;                            \
    } while (0)

    APPEND_F3(state->x_cm);
    APPEND_F3(state->y_cm);
    APPEND_F3(state->yaw_deg);
    APPEND_F3(state->lidar_x_cm);
    APPEND_F3(state->lidar_y_cm);
    APPEND_F3(state->lidar_yaw_deg);
    APPEND_F3(state->calib_x_cm);
    APPEND_F3(state->calib_y_cm);
    APPEND_F3(state->calib_yaw_deg);
    APPEND_F3(state->h30_yaw_deg);
    APPEND_F3(state->h30_x_cm);
    APPEND_F3(state->h30_y_cm);
    APPEND_F3(state->encoder_x_cm);
    APPEND_F3(state->encoder_y_cm);
    APPEND_U32(state->h30_valid ? 1U : 0U);
    APPEND_U32(state->h30_has_attitude ? 1U : 0U);
    APPEND_U32(state->lidar_valid ? 1U : 0U);
    APPEND_U32(state->lidar_online ? 1U : 0U);
    APPEND_U32(state->h30_packet_count);
    APPEND_U32(state->lidar_packet_count);
    APPEND_U32(state->h30_crc_error_count);
    APPEND_U32(state->h30_frame_error_count);
    APPEND_U32(state->lidar_checksum_error_count);
    APPEND_U32(state->lidar_frame_error_count);
    APPEND_U32(status);
    APPEND_U32(state->tick_ms);
    APPEND_U32(state->h30_rx_byte_count);
    APPEND_U32(state->h30_has_accel ? 1U : 0U);
    APPEND_U32(state->h30_last_update_ms);
    APPEND_U32(state->lidar_rx_byte_count);
    APPEND_U32(state->lidar_last_update_ms);
    APPEND_U32(state->x_raw_count);
    APPEND_U32(state->y_raw_count);
    APPEND_I32(state->x_delta_count);
    APPEND_I32(state->y_delta_count);
    APPEND_I32(state->x_total_count);
    APPEND_I32(state->y_total_count);
    APPEND_U32(state->x_index_seen ? 1U : 0U);
    APPEND_U32(state->y_index_seen ? 1U : 0U);
    APPEND_F3(state->encoder_dis_p_mm);
    APPEND_F3(state->encoder_dis_q_mm);

#undef APPEND_F3
#undef APPEND_U32
#undef APPEND_I32

    len = append_char(line, sizeof(line), len, '\r');
    len = append_char(line, sizeof(line), len, '\n');

    if (len > 0) {
        transmit((const uint8_t *)line, (uint16_t)len);
    }
}

static void send_host_pose_frame(const Locater_State_t *state)
{
    if (state == NULL) {
        return;
    }

    uint8_t frame[LOCATER_HOST_FRAME_LEN];
    float values[LOCATER_HOST_FRAME_FLOAT_COUNT] = {0.0f};
    uint8_t sum = 0U;

    get_host_pose_values(state, values);

    frame[0] = 'P';
    frame[1] = 'G';
    memcpy(&frame[2], values, sizeof(values));

    for (uint16_t i = 0U; i < LOCATER_HOST_FRAME_LEN - 1U; i++) {
        sum = (uint8_t)(sum + frame[i]);
    }
    frame[LOCATER_HOST_FRAME_LEN - 1U] = sum;

    Driver_UART_ExtTransmit(frame, (uint16_t)sizeof(frame));
}

void StartTelemetryTask(void *argument)
{
    (void)argument;

    Driver_UART_Init();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LOCATER_TELEMETRY_PERIOD_MS);

    for (;;) {
        Locater_State_t state = {0};

        Locater_GetState(&state);

#if LOCATER_TELEMETRY_DIAG_ENABLE
        send_locater_diag_csv(Driver_UART_DebugTransmit, &state);
#else
        send_locater_csv_v3(Driver_UART_DebugTransmit, &state);
#endif
        send_host_pose_frame(&state);

        vTaskDelayUntil(&last_wake, period);
    }
}
