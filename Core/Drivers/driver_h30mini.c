#include "driver_h30mini.h"
#include "locater_config.h"
#include "usart.h"
#include <string.h>

#define H30_HEADER_1                0x59U
#define H30_HEADER_2                0x53U
#define H30_FRAME_MIN_LEN           7U
#define H30_PAYLOAD_POS             5U
#define H30_FRAME_MAX_LEN           262U

#define H30_SENSOR_TEMP_ID          0x01U
#define H30_ACCEL_ID                0x10U
#define H30_GYRO_ID                 0x20U
#define H30_EULER_ID                0x40U
#define H30_SPEED_ID                0x70U

#define H30_VECTOR_LEN              12U
#define H30_NOT_MAG_FACTOR          0.000001f
#define H30_SPEED_FACTOR            0.001f

static uint8_t s_rx_byte;
static uint8_t s_frame_buffer[H30_FRAME_MAX_LEN];
static uint16_t s_frame_len;
static H30Mini_Data_t s_h30_data;

static int32_t read_i32_le(const uint8_t *data)
{
    return (int32_t)(((uint32_t)data[0]) |
                     ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 24));
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0]) | ((uint16_t)data[1] << 8));
}

static void calc_checksum(const uint8_t *data, uint16_t len, uint8_t *ck1, uint8_t *ck2)
{
    uint8_t a = 0U;
    uint8_t b = 0U;

    for (uint16_t i = 0U; i < len; i++) {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }

    *ck1 = a;
    *ck2 = b;
}

static void restart_rx_it(void)
{
    (void)HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U);
}

static void parse_payload_item(uint8_t id, const uint8_t *data, uint8_t len, H30Mini_Data_t *next)
{
    if (data == NULL || next == NULL) {
        return;
    }

    switch (id) {
    case H30_ACCEL_ID:
        if (len == H30_VECTOR_LEN) {
            next->accel_x_mps2 = (float)read_i32_le(data) * H30_NOT_MAG_FACTOR;
            next->accel_y_mps2 = (float)read_i32_le(data + 4U) * H30_NOT_MAG_FACTOR;
            next->accel_z_mps2 = (float)read_i32_le(data + 8U) * H30_NOT_MAG_FACTOR;
            next->has_accel = true;
        }
        break;

    case H30_GYRO_ID:
        if (len == H30_VECTOR_LEN) {
            next->gyro_x_dps = (float)read_i32_le(data) * H30_NOT_MAG_FACTOR;
            next->gyro_y_dps = (float)read_i32_le(data + 4U) * H30_NOT_MAG_FACTOR;
            next->gyro_z_dps = (float)read_i32_le(data + 8U) * H30_NOT_MAG_FACTOR;
            next->has_gyro = true;
        }
        break;

    case H30_EULER_ID:
        if (len == H30_VECTOR_LEN) {
            next->pitch_deg = (float)read_i32_le(data) * H30_NOT_MAG_FACTOR;
            next->roll_deg = (float)read_i32_le(data + 4U) * H30_NOT_MAG_FACTOR;
            next->yaw_deg = (float)read_i32_le(data + 8U) * H30_NOT_MAG_FACTOR;
            next->has_attitude = true;
        }
        break;

    case H30_SPEED_ID:
        if (len == H30_VECTOR_LEN) {
            next->vel_n_mps = (float)read_i32_le(data) * H30_SPEED_FACTOR;
            next->vel_e_mps = (float)read_i32_le(data + 4U) * H30_SPEED_FACTOR;
            next->vel_d_mps = (float)read_i32_le(data + 8U) * H30_SPEED_FACTOR;
            next->has_velocity = true;
        }
        break;

    case H30_SENSOR_TEMP_ID:
    default:
        break;
    }
}

static void parse_frame(const uint8_t *frame, uint16_t frame_len)
{
    uint8_t ck1 = 0U;
    uint8_t ck2 = 0U;

    if (frame == NULL || frame_len < H30_FRAME_MIN_LEN) {
        s_h30_data.frame_error_count++;
        return;
    }

    const uint8_t payload_len = frame[4U];
    const uint16_t expected_len = (uint16_t)payload_len + H30_FRAME_MIN_LEN;
    if (frame_len != expected_len) {
        s_h30_data.frame_error_count++;
        return;
    }

    calc_checksum(frame + 2U, (uint16_t)payload_len + 3U, &ck1, &ck2);
    if (ck1 != frame[5U + payload_len] || ck2 != frame[6U + payload_len]) {
        s_h30_data.crc_error_count++;
        return;
    }

    H30Mini_Data_t next = s_h30_data;
    uint16_t pos = H30_PAYLOAD_POS;
    const uint16_t payload_end = (uint16_t)H30_PAYLOAD_POS + payload_len;

    while ((uint16_t)(pos + 2U) <= payload_end) {
        const uint8_t id = frame[pos];
        const uint8_t len = frame[pos + 1U];
        pos = (uint16_t)(pos + 2U);

        if ((uint16_t)(pos + len) > payload_end) {
            next.frame_error_count++;
            break;
        }

        parse_payload_item(id, frame + pos, len, &next);
        pos = (uint16_t)(pos + len);
    }

    next.valid = true;
    next.tid = read_u16_le(frame + 2U);
    next.packet_count++;
    next.last_update_ms = HAL_GetTick();
    s_h30_data = next;
}

static void feed_byte(uint8_t byte)
{
    s_h30_data.rx_byte_count++;

    if (s_frame_len == 0U) {
        if (byte == H30_HEADER_1) {
            s_frame_buffer[s_frame_len++] = byte;
        }
        return;
    }

    if (s_frame_len == 1U) {
        if (byte == H30_HEADER_2) {
            s_frame_buffer[s_frame_len++] = byte;
        } else {
            s_frame_len = (byte == H30_HEADER_1) ? 1U : 0U;
            if (s_frame_len == 1U) {
                s_frame_buffer[0U] = H30_HEADER_1;
            }
        }
        return;
    }

    if (s_frame_len >= sizeof(s_frame_buffer)) {
        s_frame_len = 0U;
        s_h30_data.frame_error_count++;
        return;
    }

    s_frame_buffer[s_frame_len++] = byte;

    if (s_frame_len >= 5U) {
        const uint16_t expected_len = (uint16_t)s_frame_buffer[4U] + H30_FRAME_MIN_LEN;
        if (expected_len > sizeof(s_frame_buffer)) {
            s_frame_len = 0U;
            s_h30_data.frame_error_count++;
            return;
        }

        if (s_frame_len == expected_len) {
            parse_frame(s_frame_buffer, s_frame_len);
            s_frame_len = 0U;
        }
    }
}

void Driver_H30Mini_Init(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memset(&s_h30_data, 0, sizeof(s_h30_data));
    s_frame_len = 0U;
    __set_PRIMASK(primask);

    restart_rx_it();
}

void Driver_H30Mini_GetData(H30Mini_Data_t *data)
{
    if (data == NULL) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *data = s_h30_data;
    __set_PRIMASK(primask);
}

void Driver_H30Mini_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != UART4) {
        return;
    }

    feed_byte(s_rx_byte);
    restart_rx_it();
}
