#ifndef ALGO_CRC_H
#define ALGO_CRC_H

#include <stdint.h>
#include <stddef.h> // for size_t

// CRC16-CCITT
uint16_t Algo_CRC16_CCITT(uint16_t crc, uint8_t const *buffer, size_t len);

// CRC32 (用于 A1 电机)
uint32_t Algo_CRC32_Core(uint32_t* ptr, uint32_t len);

#endif // ALGO_CRC_H