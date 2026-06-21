#ifndef DRIVER_ENCODER_H
#define DRIVER_ENCODER_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t x_raw_count;
    uint32_t y_raw_count;
    int32_t x_delta_count;
    int32_t y_delta_count;
    int32_t x_total_count;
    int32_t y_total_count;
    bool x_index_seen;
    bool y_index_seen;
    bool x_pulse_seen;
    bool y_pulse_seen;
} Encoder_Snapshot_t;

void Driver_Encoder_Init(void);
void Driver_Encoder_Reset(void);
void Driver_Encoder_GetSnapshot(Encoder_Snapshot_t *snapshot);
void Driver_Encoder_ClearIndexFlags(void);
void Driver_Encoder_EXTI_Callback(uint16_t gpio_pin);

#endif /* DRIVER_ENCODER_H */
