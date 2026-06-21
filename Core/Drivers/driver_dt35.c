#include "driver_dt35.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "locater_config.h"
#include "task.h"
#include "usart.h"
#include "stm32g4xx_hal.h"
#include <string.h>

#define DT35_FUNC_READ_HOLDING_REGISTER 0x03U
#define DT35_RESPONSE_LEN               7U

typedef struct {
    uint8_t id;
    float scale;
    float offset_m;
} DT35_Calibration_t;

static DT35_Data_t s_dt35_data;

static const DT35_Calibration_t s_dt35_cal_1 = {
    .id = LOCATER_DT35_SENSOR_1_ID,
    .scale = LOCATER_DT35_SENSOR_1_SCALE,
    .offset_m = LOCATER_DT35_SENSOR_1_OFFSET_M,
};

static const DT35_Calibration_t s_dt35_cal_2 = {
    .id = LOCATER_DT35_SENSOR_2_ID,
    .scale = LOCATER_DT35_SENSOR_2_SCALE,
    .offset_m = LOCATER_DT35_SENSOR_2_OFFSET_M,
};

static uint16_t dt35_crc16(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint8_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0U; j < 8U; j++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }

    return (uint16_t)((crc >> 8U) | (crc << 8U));
}

static void dt35_clear_uart_rx(void)
{
    (void)HAL_UART_AbortReceive(&huart5);
    __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                   UART_CLEAR_NEF | UART_CLEAR_OREF |
                                   UART_CLEAR_IDLEF);

    for (uint8_t i = 0U; i < LOCATER_DT35_UART_RX_DRAIN_LIMIT; i++) {
        if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE) == RESET) {
            break;
        }
        volatile uint32_t discarded = huart5.Instance->RDR;
        (void)discarded;
    }
}

static void dt35_mark_error(DT35_SensorData_t *sensor, DT35_Error_t error)
{
    taskENTER_CRITICAL();
    sensor->last_error = (uint8_t)error;
    sensor->error_count++;
    sensor->consecutive_errors++;
    if (error == DT35_ERROR_TIMEOUT) {
        sensor->timeout_count++;
    } else if (error == DT35_ERROR_FRAME) {
        sensor->frame_error_count++;
    }
    if (sensor->consecutive_errors >= LOCATER_DT35_MAX_CONSECUTIVE_ERRORS) {
        sensor->valid = false;
    }
    taskEXIT_CRITICAL();

    dt35_clear_uart_rx();
}

static void dt35_mark_ok(DT35_SensorData_t *sensor, uint8_t active_id, uint16_t raw, float distance_mm)
{
    taskENTER_CRITICAL();
    sensor->active_id = active_id;
    sensor->raw = raw;
    sensor->distance_mm = distance_mm;
    sensor->valid = true;
    sensor->last_error = (uint8_t)DT35_ERROR_NONE;
    sensor->consecutive_errors = 0U;
    sensor->ok_count++;
    sensor->last_update_ms = HAL_GetTick();
    taskEXIT_CRITICAL();
}

static bool dt35_distance_is_valid(float distance_mm)
{
    return distance_mm >= LOCATER_DT35_MIN_DISTANCE_MM &&
           distance_mm <= LOCATER_DT35_MAX_DISTANCE_MM;
}

static void dt35_init_sensor(DT35_SensorData_t *sensor, uint8_t id)
{
    if (sensor == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    memset(sensor, 0, sizeof(*sensor));
    sensor->id = id;
    sensor->active_id = id;
    taskEXIT_CRITICAL();
}

static bool dt35_frame_is_valid(const uint8_t *frame, uint8_t expected_id)
{
    if (frame == NULL) {
        return false;
    }

    const uint16_t rx_crc = (uint16_t)((frame[5] << 8U) | frame[6]);
    return frame[0] == expected_id &&
           frame[1] == DT35_FUNC_READ_HOLDING_REGISTER &&
           frame[2] == 0x02U &&
           dt35_crc16(frame, 5U) == rx_crc;
}

static bool dt35_receive_response(DT35_SensorData_t *sensor, uint8_t expected_id, uint8_t *response)
{
    uint8_t window[DT35_RESPONSE_LEN] = {0};
    uint8_t window_len = 0U;
    const uint32_t start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) <= LOCATER_DT35_UART_RX_TIMEOUT_MS) {
        uint8_t byte = 0U;
        if (HAL_UART_Receive(&huart5, &byte, 1U, LOCATER_DT35_UART_BYTE_TIMEOUT_MS) != HAL_OK) {
            continue;
        }

        taskENTER_CRITICAL();
        sensor->rx_byte_count++;
        taskEXIT_CRITICAL();

        if (window_len < DT35_RESPONSE_LEN) {
            window[window_len++] = byte;
        } else {
            memmove(&window[0], &window[1], DT35_RESPONSE_LEN - 1U);
            window[DT35_RESPONSE_LEN - 1U] = byte;
        }

        if (window_len == DT35_RESPONSE_LEN && dt35_frame_is_valid(window, expected_id)) {
            memcpy(response, window, DT35_RESPONSE_LEN);
            return true;
        }
    }

    if (window_len >= DT35_RESPONSE_LEN) {
        if (sensor != NULL) {
            taskENTER_CRITICAL();
            sensor->frame_error_count++;
            taskEXIT_CRITICAL();
        }
    }
    return false;
}

static bool dt35_try_read_id(DT35_SensorData_t *sensor,
                             const DT35_Calibration_t *cal,
                             uint8_t id,
                             DT35_Error_t *last_error)
{
    if (sensor == NULL || cal == NULL) {
        return false;
    }

    uint8_t tx[8] = {0};
    uint8_t rx[DT35_RESPONSE_LEN] = {0};

    dt35_clear_uart_rx();

    tx[0] = id;
    tx[1] = DT35_FUNC_READ_HOLDING_REGISTER;
    tx[5] = 0x01U;

    const uint16_t tx_crc = dt35_crc16(tx, 6U);
    tx[6] = (uint8_t)((tx_crc >> 8U) & 0xFFU);
    tx[7] = (uint8_t)(tx_crc & 0xFFU);

    if (HAL_UART_Transmit(&huart5, tx, (uint16_t)sizeof(tx), LOCATER_DT35_UART_TX_TIMEOUT_MS) != HAL_OK) {
        if (last_error != NULL) {
            *last_error = DT35_ERROR_TX;
        }
        return false;
    }

    taskENTER_CRITICAL();
    sensor->tx_count++;
    taskEXIT_CRITICAL();

    if (!dt35_receive_response(sensor, id, rx)) {
        if (last_error != NULL) {
            *last_error = DT35_ERROR_TIMEOUT;
        }
        return false;
    }

    if (!dt35_frame_is_valid(rx, id)) {
        if (last_error != NULL) {
            *last_error = DT35_ERROR_FRAME;
        }
        return false;
    }

    const uint16_t raw = (uint16_t)((rx[3] << 8U) | rx[4]);
    const float distance_m = cal->scale * (float)raw + cal->offset_m;
    const float distance_mm = distance_m * 1000.0f;

    if (!dt35_distance_is_valid(distance_mm)) {
        if (last_error != NULL) {
            *last_error = DT35_ERROR_RANGE;
        }
        return false;
    }

    dt35_mark_ok(sensor, id, raw, distance_mm);
    return true;
}

static void dt35_read_sensor(DT35_SensorData_t *sensor,
                             const DT35_Calibration_t *cal,
                             uint8_t skip_scan_id)
{
    if (sensor == NULL || cal == NULL) {
        return;
    }

    DT35_Error_t last_error = DT35_ERROR_TIMEOUT;

    if (cal->id != skip_scan_id) {
        for (uint8_t attempt = 0U; attempt <= LOCATER_DT35_READ_RETRY_COUNT; attempt++) {
            if (dt35_try_read_id(sensor, cal, cal->id, &last_error)) {
                return;
            }
        }
    }

#if LOCATER_DT35_FALLBACK_SCAN_ENABLE
    if (sensor->consecutive_errors >= LOCATER_DT35_MAX_CONSECUTIVE_ERRORS) {
        for (uint8_t id = LOCATER_DT35_SCAN_MIN_ID; id <= LOCATER_DT35_SCAN_MAX_ID; id++) {
            if (id == cal->id || id == skip_scan_id) {
                continue;
            }
            if (dt35_try_read_id(sensor, cal, id, &last_error)) {
                return;
            }
        }
    }
#else
    (void)skip_scan_id;
#endif

    dt35_mark_error(sensor, last_error);
}

void Driver_DT35_Init(void)
{
    dt35_init_sensor(&s_dt35_data.sensor_1, LOCATER_DT35_SENSOR_1_ID);
    dt35_init_sensor(&s_dt35_data.sensor_2, LOCATER_DT35_SENSOR_2_ID);
    dt35_clear_uart_rx();
}

void Driver_DT35_GetData(DT35_Data_t *data)
{
    if (data == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *data = s_dt35_data;
    taskEXIT_CRITICAL();
}

void StartDT35Task(void *argument)
{
    (void)argument;

    Driver_DT35_Init();
    osDelay(LOCATER_DT35_STARTUP_DELAY_MS);

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LOCATER_DT35_TASK_PERIOD_MS);
    bool read_sensor_2_next = false;

    for (;;) {
#if LOCATER_DT35_ONLY_SENSOR_2_TEST
        dt35_read_sensor(&s_dt35_data.sensor_2, &s_dt35_cal_2, 0U);
#else
        if (read_sensor_2_next) {
            const uint8_t skip_id = s_dt35_data.sensor_1.valid ? s_dt35_data.sensor_1.active_id : 0U;
            dt35_read_sensor(&s_dt35_data.sensor_2, &s_dt35_cal_2, skip_id);
            read_sensor_2_next = false;
        } else {
            const uint8_t skip_id = s_dt35_data.sensor_2.valid ? s_dt35_data.sensor_2.active_id : 0U;
            dt35_read_sensor(&s_dt35_data.sensor_1, &s_dt35_cal_1, skip_id);
            read_sensor_2_next = true;
        }
#endif

        vTaskDelayUntil(&last_wake, period);
    }
}
