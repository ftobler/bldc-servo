/**
@file crc16.h
@brief Include file for crc.c

Contains defines for initial values and polynomials.

*/


#ifndef CRC16_H__
#define CRC16_H__


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stddef.h>


// For information only, table is hardcoded to P_16 = 0xA001
#define CRC16_INITIAL_VALUE   0xFFFF
#define P_16                  0xA001

#define OPTION__TableCRC 1


/**
 *  @brief Add a byte to a CRC16
 *
 *  @param [in] CRC The ongoing CRC value
 *  @param [in] Data the byte to add to the CRC
 *  @return the updated CRC value
 *
 *  @details Details
 */
uint16_t update_CRC16(uint16_t crc, uint8_t data);

/**
 *  @brief Calculate a CRC16 over the byte buffer.
 *
 *  @param [in] ptr pointer to a buffer of bytes.
 *  @param [in] length number of bytes to include in crc
 *  @return The CRC16 value
 *
 *  @details
 */
uint16_t calculate_CRC16(const uint8_t* ptr, size_t length);


#ifdef __cplusplus
}
#endif


#endif  /* CRC16_H__ */
