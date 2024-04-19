/*
 * protocol.h
 *
 *  Created on: Apr 5, 2024
 *      Author: cchtofl01
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif


void protocol_init();
void protocol_loop();
void protocol_uart_isr();


#ifdef __cplusplus
}
#endif


#endif /* PROTOCOL_H_ */
