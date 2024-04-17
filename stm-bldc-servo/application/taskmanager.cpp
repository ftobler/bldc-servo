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

void taskmanager_init() {
	application_setup();
	while (1) {
		application_loop();
	}
}


