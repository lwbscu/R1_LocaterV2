#ifndef TASK_LOCATER_H
#define TASK_LOCATER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t tick_ms;
    uint32_t x_raw_count;
    uint32_t y_raw_count;
    int32_t x_delta_count;
    int32_t y_delta_count;
    int32_t x_total_count;
    int32_t y_total_count;
    bool x_index_seen;
    bool y_index_seen;
    bool h30_valid;
    bool h30_has_attitude;
    bool h30_has_accel;
    uint32_t h30_packet_count;
    uint32_t h30_rx_byte_count;
    uint32_t h30_crc_error_count;
    uint32_t h30_frame_error_count;
    uint32_t h30_last_update_ms;
    float yaw_deg;
    float h30_x_cm;
    float h30_y_cm;
    float h30_vx_mps;
    float h30_vy_mps;
    float h30_accel_x_mps2;
    float h30_accel_y_mps2;
    float h30_gyro_z_dps;
    float encoder_x_cm;
    float encoder_y_cm;
    float encoder_dis_p_mm;
    float encoder_dis_q_mm;
} Locater_State_t;

void StartLocaterTask(void *argument);
void Locater_GetState(Locater_State_t *state);
void Locater_RequestZero(void);

#endif /* TASK_LOCATER_H */
