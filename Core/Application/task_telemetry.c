#include "task_locater.h"
#include "FreeRTOS.h"
#include "task.h"
#include "driver_uart.h"
#include "locater_config.h"
#include <stdio.h>

typedef void (*TelemetryTransmit_t)(const uint8_t *data, uint16_t len);

static void send_firewater_values(TelemetryTransmit_t transmit, const float *values, uint16_t count);

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

static void send_firewater_values(TelemetryTransmit_t transmit, const float *values, uint16_t count)
{
    if (transmit == NULL || values == NULL || count == 0U) {
        return;
    }

    char line[128];
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

void StartTelemetryTask(void *argument)
{
    (void)argument;

    Driver_UART_Init();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LOCATER_TELEMETRY_PERIOD_MS);

    for (;;) {
        Locater_State_t state = {0};

        Locater_GetState(&state);

#if LOCATER_TELEMETRY_H30_DIAG_ENABLE
        const float values[] = {
            (float)state.h30_rx_byte_count,
            (float)state.h30_packet_count,
            state.h30_has_attitude ? 1.0f : 0.0f,
            (float)state.h30_crc_error_count,
            (float)state.h30_frame_error_count,
            state.yaw_deg,
            state.encoder_x_cm,
            state.encoder_y_cm,
        };
        send_firewater_values(Driver_UART_DebugTransmit, values, (uint16_t)(sizeof(values) / sizeof(values[0])));
#else
        const float values[] = {
            state.yaw_deg,
            state.h30_x_cm,
            state.h30_y_cm,
            state.encoder_x_cm,
            state.encoder_y_cm,
        };
        send_firewater_values(Driver_UART_DebugTransmit, values, (uint16_t)(sizeof(values) / sizeof(values[0])));
#endif

        vTaskDelayUntil(&last_wake, period);
    }
}
