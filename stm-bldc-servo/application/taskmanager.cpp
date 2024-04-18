/*
 * taskmanager.cpp
 *
 *  Created on: Apr 17, 2024
 *      Author: ftobler
 */

#include "taskmanager.h"
#include "flortos.h"
#include "stm32hal.h"
#include "application.h"
#include "main.h"
#include "stdlib.h"


enum {
	STD_STACK_SIZE = 1024
};

static uint8_t stack_idle_task[STD_STACK_SIZE];
static uint8_t stack_application_task[STD_STACK_SIZE];
static uint8_t stack_controller_task[STD_STACK_SIZE];
static uint8_t stack_communication_task[STD_STACK_SIZE];

static void idle_task_fn();
static void application_task_fn();
static void communication_task_fn();
static void controller_task_fn();
extern uint32_t angle_target;
uint32_t enable_automatic = 0;

void taskmanager_init() {
	scheduler_init();
	scheduler_addTask(TASK_ID_IDLE, idle_task_fn, stack_idle_task, STD_STACK_SIZE); //lowest priority
	scheduler_addTask(TASK_ID_APPLICATION, application_task_fn, stack_application_task, STD_STACK_SIZE);
	scheduler_addTask(TASK_ID_COMMUNICATION, communication_task_fn, stack_communication_task, STD_STACK_SIZE);
	scheduler_addTask(TASK_ID_CONTROLLER, controller_task_fn, stack_controller_task, STD_STACK_SIZE); //highest priority
	scheduler_join();
}


static void idle_task_fn() {
	while (1) {
	}
}

static void application_task_fn() {
	application_setup();
	while (1) {
		scheduler_event_wait(EVENT_APPLICATION_TIMER);
		control_loop();
		application_loop();
	}
}

static void communication_task_fn() {
	while (1) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (GPIO_PinState)((uwTick % 500) > 250));
		scheduler_task_sleep(11);
	}
}

static void controller_task_fn() {
	srand(0);
	while (1) {
		scheduler_task_sleep(2000);
		if (enable_automatic || 1) {
//			angle_target = 1000;
			angle_target = 500 + rand() / 143165;
			scheduler_task_sleep(1000);
			angle_target = 500 + rand() / 143165;
//			angle_target = 10000;
		}
	}
}



