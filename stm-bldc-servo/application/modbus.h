/*
 * modbus.h
 *
 *  Created on: Feb 4, 2022
 *      Author: cchtofl01
 */

#ifndef MODBUS_H_
#define MODBUS_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "serial.h"


void modbus_init_slave(uint8_t deviceaddress);
void modbus_process_data_slave(Serial& serial, uint8_t* messageBuffer, uint32_t len);



#ifdef __cplusplus
}
#endif


#endif /* MODBUS_H_ */
