/*
 * serial.cpp
 *
 *  Created on: Feb 3, 2022
 *      Author: cchtofl01
 */

#include "serial.h"
#include "stm32_hal.h"



Buffer::Buffer() {
	init();
}

void Buffer::init() {
	head = 0;
	tail = 0;
}

inline uint8_t Buffer::getByte() {
	uint32_t _tail = tail;
	uint8_t data = buf[_tail];
	tail = (_tail + 1) % BUFFER_LEN;
	return data;
}
inline uint32_t Buffer::getAvailable() {
	return (head - tail) % BUFFER_LEN;
}
inline void Buffer::setByte(uint8_t data) {
	uint32_t _head = head;
	buf[_head] = data;
	head = (_head + 1) % BUFFER_LEN;
}






void Serial::ISR(UART_HandleTypeDef* handler) {
	uint32_t isrflags   = READ_REG(huart->Instance->ISR);
	uint32_t cr1its     = READ_REG(huart->Instance->CR1);
//	uint32_t cr3its     = READ_REG(huart->Instance->CR3);


	uint32_t errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF));
	if (errorflags == 0) {
		//handle normal
		if (((isrflags & USART_ISR_RXNE_RXFNE) != 0U)
				&& (((cr1its & USART_ISR_RXNE_RXFNE) != 0U)   )) {
			//received something
			uint16_t uhMask = huart->Mask;
			uint16_t uhdata = (uint16_t) READ_REG(huart->Instance->RDR);
			uint8_t data = (uint8_t)(uhdata & (uint8_t)uhMask);
			in.setByte(data);
			if (hook_byte_received) {
				hook_byte_received();
			}
		}
	}

	if (((isrflags & USART_ISR_TXE_TXFNF) != 0U)
		      && (((cr1its & USART_ISR_TXE_TXFNF) != 0U)   )) {
		uint8_t outAvail = out.getAvailable();
		if (outAvail == 0) {

			/* Disable the UART Transmit Data Register Empty Interrupt */
			ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE_TXFNFIE);

			/* Enable the UART Transmit Complete Interrupt */
			SET_BIT(huart->Instance->CR1, USART_CR1_TCIE);
		} else {
			huart->Instance->TDR = out.getByte();
		}
    }

	if (((isrflags & USART_ISR_TC) != 0U) && ((cr1its & USART_CR1_TCIE) != 0U)) {
		//transmission ended

		/* Disable the UART Transmit Complete Interrupt */
		ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);

		this->txnComplete = 1;

		if (hook_end_transmission) {
			hook_end_transmission();
		}
	}

	if ((isrflags & USART_ISR_RTOF) != 0U){
		//interrupt when a timeout occures for busses like modbus.
		//for modbus this is 3.5 characters long.
		if (hook_end_transmission_modbus) {
			hook_end_transmission_modbus();
		}
	}

	huart->Instance->ICR = 0b11111111111111111111111111111111;   //clear all flags
}


/**
 * constructor. Make sure huart pointer is set to 0
 * so errors can be handled
 */
Serial::Serial() {
	huart = 0;
	hook_begin_transmission = 0;
	hook_byte_received = 0;
	hook_end_transmission = 0;
	hook_end_transmission_modbus = 0;
}


void Serial::init(UART_HandleTypeDef* handler) {
	huart = handler;
	in.init();
	out.init();
//	this->flow = 0;

	UART_HandleTypeDef *huart = handler;
	/* Computation of UART mask to apply to RDR register */
	UART_MASK_COMPUTATION(huart);

	huart->ErrorCode = HAL_UART_ERROR_NONE;
	huart->RxState = HAL_UART_STATE_BUSY_RX;

	huart->gState = HAL_UART_STATE_READY;

	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

	/* Enable the UART Parity Error interrupt and Data Register Not Empty interrupt */
	SET_BIT(huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE_RXFNEIE);
}


void Serial::initFlow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	flow = 1;
	flowPort = GPIOx;
	flowPin = GPIO_Pin;
}


uint32_t Serial::available(){
	return in.getAvailable();
}


void Serial::flushRX(){
	in.init();
}


void Serial::flushTX(){
	out.init();
}


void Serial::print(const char* str){
	const uint8_t* ptr = (uint8_t*)str;
	while (*ptr != '\0'){
		out.setByte(*ptr);
		ptr++;
	}
	enableTx();
}


/**
 * unguarded about overflow!
 * use Serial_available!
 */
uint8_t Serial::read() {
	return in.getByte();
}


uint32_t Serial::readBuf(uint8_t* buf, uint16_t len) {
	uint16_t count = 0;
	while (in.getAvailable() && (count < len)) {
		buf[count] = in.getByte();
		count++;
	}
	return count;
}


void Serial::writeBuf(const uint8_t* buf, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		out.setByte(buf[i]);
	}
	enableTx();
}


void Serial::write(const uint8_t data) {
	out.setByte(data);
	enableTx();
}


void Serial::enableTx() {
	if (hook_begin_transmission) {
		hook_begin_transmission();
	}

	/* Enable the Transmit Data Register Empty interrupt */
	ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_TXEIE_TXFNFIE);

	txnComplete = 0;
}

