#include "driver_uart.h"
#include "driver_h30mini.h"
#include "driver_lidar_pose.h"
#include "usart.h"

#define DEBUG_LINE_BUFFER_SIZE 128U

static Driver_UART_LineCallback_t s_debug_callback;
static uint8_t s_debug_rx_byte;
static uint8_t s_debug_line[DEBUG_LINE_BUFFER_SIZE];
static uint16_t s_debug_line_len;

void Driver_UART_Init(void)
{
    s_debug_line_len = 0U;
    (void)HAL_UART_Receive_IT(&huart1, &s_debug_rx_byte, 1U);
}

void Driver_UART_RegisterDebugCallback(Driver_UART_LineCallback_t callback)
{
    s_debug_callback = callback;
}

void Driver_UART_DebugTransmit(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 20U);
}

void Driver_UART_IMUTransmit(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }
    (void)HAL_UART_Transmit(&huart4, (uint8_t *)data, len, 20U);
}

void Driver_UART_ExtTransmit(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 20U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        Driver_H30Mini_RxCpltCallback(huart);
        return;
    }

    if (huart->Instance == USART3) {
        Driver_LidarPose_RxCpltCallback(huart);
        return;
    }

    if (huart->Instance != USART1) {
        return;
    }

    if (s_debug_rx_byte == 'R' || s_debug_rx_byte == 'r') {
        const uint8_t command = s_debug_rx_byte;

        s_debug_line_len = 0U;
        if (s_debug_callback != NULL) {
            s_debug_callback(&command, 1U);
        }
    } else if (s_debug_rx_byte == '\n' || s_debug_rx_byte == '\r') {
        if (s_debug_line_len > 0U && s_debug_callback != NULL) {
            s_debug_callback(s_debug_line, s_debug_line_len);
        }
        s_debug_line_len = 0U;
    } else if (s_debug_line_len < sizeof(s_debug_line)) {
        s_debug_line[s_debug_line_len++] = s_debug_rx_byte;
    } else {
        s_debug_line_len = 0U;
    }

    (void)HAL_UART_Receive_IT(&huart1, &s_debug_rx_byte, 1U);
}
