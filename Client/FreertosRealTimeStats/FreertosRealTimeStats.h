/*
 * FreertosRealTimeStats.h
 *
 *  Created on: Apr 27, 2025
 *      Author: gdurand
 */

#ifndef FREERTOSREALTIMESTATS_FREERTOSREALTIMESTATS_H_
#define FREERTOSREALTIMESTATS_FREERTOSREALTIMESTATS_H_

#define ARRAY_SIZE_OFFSET   5   //Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE

typedef enum {
	STM32_OK = 0, STM32_ERR_NO_MEM, STM32_ERR_INVALID_SIZE, STM32_ERR_INVALID_STATE
} stm32_err_t;

stm32_err_t print_real_time_stats(TickType_t xTicksToWait);

#endif /* FREERTOSREALTIMESTATS_FREERTOSREALTIMESTATS_H_ */
