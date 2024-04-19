/*
 * protocol.c
 *
 *  Created on: Apr 5, 2024
 *      Author: cchtofl01
 */




#include "protocol.h"
#include "stm32_hal.h"
#include "flortos.h"
#include "taskmanager.h"
#include "serial.h"
#include "modbus.h"


extern UART_HandleTypeDef huart2;

static void hook_end_transmission();

static Serial serial;
static uint8_t uart_receive[256];


void protocol_init() {
	HAL_UART_ReceiverTimeout_Config(&huart2, 40); //transmission timeout timer value. based on baudrate. for 3.5words use 3.5*11bit = 38.5
	HAL_UART_EnableReceiverTimeout(&huart2);
	serial.init(&huart2);
	serial.hook_end_transmission_modbus = hook_end_transmission;

	//enable receive timeout interrupt
	SET_BIT(huart2.Instance->CR1, USART_CR1_RTOIE);

	modbus_init_slave(10);
}

void protocol_loop() {
	int len = serial.available();
	serial.readBuf(uart_receive, len);

	modbus_process_data_slave(serial, uart_receive, len);
}

/**
 * UART interrupt (C-style) to be passed to C++ object.
 */
void protocol_uart_isr() {
	serial.ISR(&huart2);
}

/**
 * Hook event on end transmission interrupt (ISR context)
 */
static void hook_end_transmission() {
	scheduler_event_set(TASK_ID_PROTOCOL, EVENT_PROTOCOL_UART_IDLE);
}
