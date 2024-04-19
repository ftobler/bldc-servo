/*
 * Serialh
 *
 *  Created on: 17.10.2020
 *      Author: Florin
 */

#ifndef SERIAL_H_
#define SERIAL_H_


#include "stdint.h"
#include "stm32_hal.h"


enum {
	BUFFER_LEN = 256
};

typedef void (*serial_hook)();


class Buffer {
private:
	uint32_t head;
	uint32_t tail;
	uint8_t buf[BUFFER_LEN];
public:
	Buffer();
	inline uint8_t getByte();
	void init();
	inline uint32_t getAvailable();
	inline void setByte(uint8_t data);
};


/**
 * UART implementation for STM32G0 MCUs
 * the implementation uses a send and receive FIFO buffer
 */
class Serial {
private:
	Buffer out;
	Buffer in;
	uint8_t txnComplete;
	uint8_t flow;
	GPIO_TypeDef* flowPort;
	uint16_t flowPin;
	void enableTx();
public:
	UART_HandleTypeDef* huart; //need access to this for error handling
	serial_hook hook_begin_transmission;
	serial_hook hook_byte_received;
	serial_hook hook_end_transmission;
	serial_hook hook_end_transmission_modbus;
	Serial();
	void ISR(UART_HandleTypeDef* handler);  //must pass handler every time to avoid having unitialized stuff
	void init(UART_HandleTypeDef* handler);
	void initFlow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	uint32_t available();
	void flushRX();
	void flushTX();
	void print(const char* str);
	uint8_t read();
	uint32_t readBuf(uint8_t* buf, uint16_t len);
	void writeBuf(const uint8_t* buf, uint16_t len);
	void write(const uint8_t data);
};


#endif /* SERIAL_H_ */
