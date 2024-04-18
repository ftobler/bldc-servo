/*
 * application.cpp
 *
 *  Created on: Apr 17, 2024
 *      Author: ftobler
 */


#include "application.h"
#include "stm32hal.h"
#include "main.h"
#include "flortos.h"

enum {
	N = 256,
   AS5600L_ADDR = 0x80,

   PWM_MID = 1600,
   AMP_MAX = 1550,
};

constexpr uint16_t sintab[N+1] = {0, 402, 804, 1206, 1608, 2010, 2412, 2813, 3215, 3617, 4018, 4419, 4821, 5221, 5622, 6023, 6423,
		6823, 7223, 7622, 8022, 8421, 8819, 9218, 9615, 10013, 10410, 10807, 11203, 11599, 11995, 12390, 12785, 13179, 13573,
		13966, 14358, 14750, 15142, 15533, 15923, 16313, 16702, 17091, 17479, 17866, 18252, 18638, 19023, 19408, 19791, 20174,
		20557, 20938, 21319, 21699, 22078, 22456, 22833, 23210, 23585, 23960, 24334, 24707, 25079, 25450, 25820, 26189, 26557,
		26924, 27290, 27655, 28019, 28382, 28744, 29105, 29465, 29823, 30181, 30537, 30892, 31247, 31599, 31951, 32302, 32651,
		32999, 33346, 33691, 34035, 34378, 34720, 35061, 35400, 35737, 36074, 36409, 36742, 37075, 37406, 37735, 38063, 38390,
		38715, 39039, 39361, 39682, 40001, 40319, 40635, 40950, 41263, 41574, 41885, 42193, 42500, 42805, 43109, 43411, 43711,
		44010, 44307, 44603, 44896, 45189, 45479, 45768, 46055, 46340, 46623, 46905, 47185, 47463, 47739, 48014, 48287, 48558,
		48827, 49094, 49360, 49623, 49885, 50145, 50403, 50659, 50913, 51165, 51415, 51664, 51910, 52155, 52397, 52638, 52876,
		53113, 53347, 53580, 53810, 54039, 54265, 54490, 54712, 54933, 55151, 55367, 55581, 55793, 56003, 56211, 56416, 56620,
		56821, 57021, 57218, 57413, 57606, 57796, 57985, 58171, 58355, 58537, 58717, 58894, 59069, 59242, 59413, 59582, 59748,
		59912, 60074, 60234, 60391, 60546, 60699, 60849, 60997, 61143, 61287, 61428, 61567, 61704, 61838, 61970, 62100, 62227,
		62352, 62474, 62595, 62713, 62828, 62941, 63052, 63161, 63267, 63370, 63472, 63570, 63667, 63761, 63853, 63942, 64029,
		64114, 64196, 64275, 64353, 64427, 64500, 64570, 64637, 64702, 64765, 64825, 64883, 64938, 64991, 65042, 65090, 65135,
		65178, 65219, 65257, 65293, 65326, 65357, 65385, 65411, 65435, 65456, 65474, 65490, 65504, 65515, 65523, 65530, 65533,
		65535};

//uint8_t halltab[1024] = {0xFF};


extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;

static int32_t sine(uint32_t w);

volatile uint16_t commutation_debug = 0;
volatile uint32_t angle_target = 500;
volatile int32_t controlsignal = 0;

//uint8_t received[2] = {0};
uint32_t adc_dma_results[1];

volatile uint32_t perf_loop = 0;



static volatile int32_t integrator = 0;
static volatile int32_t integrator_max = 5000;
static volatile int32_t controller_p = 3;
static volatile int32_t controller_i = 15;
static volatile int32_t error_trace = 0;
static volatile float angle_target_smooth = 0;
static volatile float acceleration = 0.0002f;
static volatile float velocity = 0.0f;
static volatile float velocity_max = 0.7f;

void application_setup() {

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	htim3.Instance->CCR1 = 0;
	htim3.Instance->CCR2 = 0;
	htim3.Instance->CCR3 = 0;

	SKIP_GPIO_Port->BSRR = SKIP_Pin;


	HAL_ADC_Start_DMA(&hadc1, adc_dma_results, 1);

	scheduler_task_sleep(5);
	angle_target_smooth = adc_dma_results[0];
}

__attribute__((optimize("Ofast"))) void application_loop() {
	perf_loop++;

	uint16_t sens1 = GPIOA->IDR & SENS1_Pin;
	uint16_t sens2 = GPIOA->IDR & SENS2_Pin;
	uint16_t sens3 = GPIOA->IDR & SENS3_Pin;

	int32_t commutation = 0;
	if (sens1==0 && sens2==0 && sens3   ) commutation = 0;
	if (sens1==0 && sens2    && sens3   ) commutation = 1;
	if (sens1==0 && sens2    && sens3==0) commutation = 2;
	if (sens1    && sens2    && sens3==0) commutation = 3;
	if (sens1    && sens2==0 && sens3==0) commutation = 4;
	if (sens1    && sens2==0 && sens3   ) commutation = 5;
	commutation_debug = commutation;
	commutation = -commutation;

	int32_t block = 0;
	if (controlsignal > 0) {
		block = -controlsignal;
		commutation = (commutation + 10) % 6; //forward
	} else {
		block = controlsignal;
	    commutation = (commutation + 7) % 6; //reverse
	}
	if (block > AMP_MAX) {
		block = AMP_MAX;
	}
	if (block < -AMP_MAX) {
		block = -AMP_MAX;
	}
	switch (commutation) {
	case 0:
		htim3.Instance->CCR1 = PWM_MID + 0;
		htim3.Instance->CCR2 = PWM_MID - block;
		htim3.Instance->CCR3 = PWM_MID + block;
		break;
	case 1:
		htim3.Instance->CCR1 = PWM_MID + block;
		htim3.Instance->CCR2 = PWM_MID - block;
		htim3.Instance->CCR3 = PWM_MID + 0;
		break;
	case 2:
		htim3.Instance->CCR1 = PWM_MID + block;
		htim3.Instance->CCR2 = PWM_MID + 0;
		htim3.Instance->CCR3 = PWM_MID - block;
		break;
	case 3:
		htim3.Instance->CCR1 = PWM_MID + 0;
		htim3.Instance->CCR2 = PWM_MID + block;
		htim3.Instance->CCR3 = PWM_MID - block;
		break;
	case 4:
		htim3.Instance->CCR1 = PWM_MID - block;
		htim3.Instance->CCR2 = PWM_MID + block;
		htim3.Instance->CCR3 = PWM_MID + 0;
		break;
	case 5:
		htim3.Instance->CCR1 = PWM_MID - block;
		htim3.Instance->CCR2 = PWM_MID + 0;
		htim3.Instance->CCR3 = PWM_MID + block;
		break;
	}

}


__attribute__((optimize("Ofast"))) void control_loop() {
	//calculate the distance to target position
	float disttogo = angle_target - angle_target_smooth;

	//calculate the maximum velocity for distance to target. We don't want to be
	//faster or braking will become an issue.
	//Make this square to not need to calculate squareroot.
	float max_velocity_sq = (2 * acceleration * disttogo);

	//need to decide from which direction we make our ramps
	if (max_velocity_sq >= 0) {
		//forward direction
		if (max_velocity_sq >= velocity*velocity) {
			//accelerate
			if (velocity < velocity_max) {
				velocity += acceleration;
			}
		} else {
			//de-accelerate
			if (velocity > 0) {
				velocity -= acceleration;
			} else {
				velocity = 0;
			}
	    }
	} else {
		//backward direction
		if (-max_velocity_sq >= velocity*velocity) {
			//accelerate (reverse direction)
			if (velocity > -velocity_max) {
				velocity -= acceleration;
			}
		} else {
			//de-accelerate (reverse direction)
			if (velocity < 0) {
				velocity += acceleration;
			} else {
				velocity = 0;
			}
		}
	}
	angle_target_smooth += velocity;


	int32_t error = angle_target_smooth - adc_dma_results[0];
	integrator += error;
	if (integrator > integrator_max) {
		integrator = integrator_max;
	}
	if (integrator < -integrator_max) {
		integrator = -integrator_max;
	}

	controlsignal  = error * controller_p + integrator * controller_i / 2048;


	error_trace = (error_trace +  error * error) * 255 / 256;
}


static int32_t sine(uint32_t w) {
	w = w % (N*4);
	if (w > N*3) {
		return -sintab[N*4 - w];
	} else if (w > N*2) {
		return -sintab[w - N*2];
	} else if (w > N) {
		return sintab[N*2 - w];
	} else {
		return sintab[w];
	}
}
