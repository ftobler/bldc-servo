/*
 * taskmanager.h
 *
 *  Created on: Apr 17, 2024
 *      Author: ftobler
 */

#ifndef TASKMANAGER_H_
#define TASKMANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif


enum {
	TASK_ID_IDLE = 0,


	TASK_ID_PROTOCOL = 1,
	EVENT_PROTOCOL_UART_IDLE = 0x01,

	TASK_ID_APPLICATION = 2,
	EVENT_APPLICATION_TIMER = 0x01,

	TASK_ID_CONTROLLER = 3,
};


void taskmanager_init();

#ifdef __cplusplus
}
#endif


#endif /* TASKMANAGER_H_ */
