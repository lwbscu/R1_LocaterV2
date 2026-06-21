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
    bool x_pulse_seen;
    bool y_pulse_seen;
    bool h30_valid;
    bool h30_has_attitude;
    bool h30_has_accel;
    uint32_t h30_packet_count;
    uint32_t h30_rx_byte_count;
    uint32_t h30_crc_error_count;
    uint32_t h30_frame_error_count;
    uint32_t h30_last_update_ms;
    bool lidar_valid;
    bool lidar_online;
    uint32_t lidar_packet_count;
    uint32_t lidar_rx_byte_count;
    uint32_t lidar_checksum_error_count;
    uint32_t lidar_frame_error_count;
    uint32_t lidar_last_update_ms;
    float x_cm;
    float y_cm;
    float yaw_deg;
    float h30_yaw_deg;
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
    float calib_x_cm;
    float calib_y_cm;
    float calib_yaw_deg;
    float lidar_x_cm;
    float lidar_y_cm;
    float lidar_yaw_deg;
    bool dt35_1_valid;
    bool dt35_2_valid;
    uint16_t dt35_1_raw;
    uint16_t dt35_2_raw;
    uint32_t dt35_1_ok_count;
    uint32_t dt35_2_ok_count;
    uint32_t dt35_1_error_count;
    uint32_t dt35_2_error_count;
    uint32_t dt35_1_last_update_ms;
    uint32_t dt35_2_last_update_ms;
    float dt35_1_mm;
    float dt35_2_mm;
} Locater_State_t;

void StartLocaterTask(void *argument);
void Locater_GetState(Locater_State_t *state);
void Locater_RequestZero(void);

#endif /* TASK_LOCATER_H */
