#ifndef DRIVER_LIDAR_POSE_H
#define DRIVER_LIDAR_POSE_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    bool has_pose;
    uint32_t packet_count;
    uint32_t rx_byte_count;
    uint32_t checksum_error_count;
    uint32_t frame_error_count;
    uint32_t last_update_ms;
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_yaw;
    float sensor_x_cm;
    float sensor_y_cm;
    float yaw_deg;
} LidarPose_Data_t;

void Driver_LidarPose_Init(void);
void Driver_LidarPose_GetData(LidarPose_Data_t *data);
void Driver_LidarPose_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* DRIVER_LIDAR_POSE_H */
