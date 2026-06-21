#ifndef DRIVER_DT35_H
#define DRIVER_DT35_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DT35_ERROR_NONE = 0U,
    DT35_ERROR_TX = 1U,
    DT35_ERROR_TIMEOUT = 2U,
    DT35_ERROR_FRAME = 3U,
    DT35_ERROR_RANGE = 4U,
} DT35_Error_t;

typedef struct {
    uint8_t id;
    uint8_t active_id;
    uint8_t last_error;
    uint8_t reserved;
    uint16_t raw;
    float distance_mm;
    bool valid;
    uint32_t ok_count;
    uint32_t error_count;
    uint32_t consecutive_errors;
    uint32_t last_update_ms;
    uint32_t tx_count;
    uint32_t rx_byte_count;
    uint32_t timeout_count;
    uint32_t frame_error_count;
} DT35_SensorData_t;

typedef struct {
    DT35_SensorData_t sensor_1;
    DT35_SensorData_t sensor_2;
} DT35_Data_t;

void Driver_DT35_Init(void);
void Driver_DT35_GetData(DT35_Data_t *data);
void StartDT35Task(void *argument);

#endif /* DRIVER_DT35_H */
