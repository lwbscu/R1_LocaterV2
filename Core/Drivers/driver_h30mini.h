#ifndef DRIVER_H30MINI_H
#define DRIVER_H30MINI_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    bool has_accel;
    bool has_gyro;
    bool has_attitude;
    bool has_velocity;
    uint16_t tid;
    uint32_t packet_count;
    uint32_t rx_byte_count;
    uint32_t crc_error_count;
    uint32_t frame_error_count;
    uint32_t last_update_ms;
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float pitch_deg;
    float roll_deg;
    float yaw_deg;
    float vel_n_mps;
    float vel_e_mps;
    float vel_d_mps;
} H30Mini_Data_t;

void Driver_H30Mini_Init(void);
void Driver_H30Mini_GetData(H30Mini_Data_t *data);
void Driver_H30Mini_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* DRIVER_H30MINI_H */
