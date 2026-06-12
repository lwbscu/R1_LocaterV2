#include "driver_lidar_pose.h"
#include "locater_config.h"
#include "usart.h"

#define LIDAR_POSE_FRAME_LEN       10U
#define LIDAR_POSE_HEADER_0        0x0FU
#define LIDAR_POSE_HEADER_1        0xF0U

static uint8_t s_rx_byte;
static uint8_t s_frame[LIDAR_POSE_FRAME_LEN];
static uint8_t s_frame_index;
static LidarPose_Data_t s_lidar_data;

static float angle_normal_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle <= -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static uint8_t frame_checksum(const uint8_t *frame)
{
    uint8_t sum = 0U;

    for (uint8_t i = 0U; i < LIDAR_POSE_FRAME_LEN - 1U; i++) {
        sum = (uint8_t)(sum + frame[i]);
    }

    return sum;
}

static int16_t read_i16_be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static void restart_rx_it(void)
{
    (void)HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1U);
}

static void process_frame(const uint8_t *frame)
{
    if (frame_checksum(frame) != frame[LIDAR_POSE_FRAME_LEN - 1U]) {
        s_lidar_data.checksum_error_count++;
        return;
    }

    const int16_t raw_x = read_i16_be(&frame[3]);
    const int16_t raw_y = read_i16_be(&frame[5]);
    const int16_t raw_yaw = read_i16_be(&frame[7]);

    s_lidar_data.raw_x = raw_x;
    s_lidar_data.raw_y = raw_y;
    s_lidar_data.raw_yaw = raw_yaw;
    s_lidar_data.sensor_x_cm = -(float)raw_y / 10.0f;
    s_lidar_data.sensor_y_cm = (float)raw_x / 10.0f;
    s_lidar_data.yaw_deg = angle_normal_deg(((float)raw_yaw * 0.001f) *
                                            (180.0f / LOCATER_PI) +
                                            LOCATER_LIDAR_YAW_OFFSET_DEG);
    s_lidar_data.valid = true;
    s_lidar_data.has_pose = true;
    s_lidar_data.packet_count++;
    s_lidar_data.last_update_ms = HAL_GetTick();
}

static void feed_byte(uint8_t byte)
{
    s_lidar_data.rx_byte_count++;

    if (s_frame_index == 0U) {
        if (byte == LIDAR_POSE_HEADER_0) {
            s_frame[s_frame_index++] = byte;
        }
        return;
    }

    if (s_frame_index == 1U) {
        if (byte == LIDAR_POSE_HEADER_1) {
            s_frame[s_frame_index++] = byte;
        } else {
            s_lidar_data.frame_error_count++;
            s_frame_index = (byte == LIDAR_POSE_HEADER_0) ? 1U : 0U;
            s_frame[0] = byte;
        }
        return;
    }

    s_frame[s_frame_index++] = byte;
    if (s_frame_index >= LIDAR_POSE_FRAME_LEN) {
        process_frame(s_frame);
        s_frame_index = 0U;
    }
}

void Driver_LidarPose_Init(void)
{
    s_rx_byte = 0U;
    s_frame_index = 0U;
    s_lidar_data = (LidarPose_Data_t){0};
    restart_rx_it();
}

void Driver_LidarPose_GetData(LidarPose_Data_t *data)
{
    if (data == NULL) {
        return;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *data = s_lidar_data;
    if (primask == 0U) {
        __enable_irq();
    }
}

void Driver_LidarPose_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != USART3) {
        return;
    }

    feed_byte(s_rx_byte);
    restart_rx_it();
}
