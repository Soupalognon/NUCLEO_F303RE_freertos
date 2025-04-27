/*
 * FreertosRealTimeStats.c
 *
 *  Created on: Apr 27, 2025
 *      Author: gdurand
 */

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include "FreertosRealTimeStats.h"
#include "../Serial/UartSerial.h"

extern UartSerial uartStream;


stm32_err_t print_real_time_stats(TickType_t xTicksToWait) {
	TaskStatus_t *start_array = NULL, *end_array = NULL;
	UBaseType_t start_array_size, end_array_size;
	uint32_t start_run_time, end_run_time;
	stm32_err_t ret;

	//Allocate array to store current task states
	start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
	start_array = (TaskStatus_t*) malloc(sizeof(TaskStatus_t) * start_array_size);
	if (start_array == NULL) {
		ret = STM32_ERR_NO_MEM;
		free(start_array);
		free(end_array);
		return ret;
	}
	//Get current task states
	start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
	if (start_array_size == 0) {
		ret = STM32_ERR_INVALID_SIZE;
		free(start_array);
		free(end_array);
		return ret;
	}

	vTaskDelay(xTicksToWait);

	//Allocate array to store tasks states post delay
	end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
	end_array = (TaskStatus_t*) malloc(sizeof(TaskStatus_t) * end_array_size);
	if (end_array == NULL) {
		ret = STM32_ERR_NO_MEM;
		free(start_array);
		free(end_array);
		return ret;
	}
	//Get post delay task states
	end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
	if (end_array_size == 0) {
		ret = STM32_ERR_INVALID_SIZE;
		free(start_array);
		free(end_array);
		return ret;
	}

	//Calculate total_elapsed_time in units of run time stats clock period.
	uint32_t total_elapsed_time = (end_run_time - start_run_time);
	if (total_elapsed_time == 0) {
		ret = STM32_ERR_INVALID_STATE;
		free(start_array);
		free(end_array);
		return ret;
	}

	uartStream.addInfo("| Task         | Run Time | Percentage\n");
	//Match each task in start_array to those in the end_array
	for (int i = 0; i < (int) start_array_size; i++) {
		int k = -1;
		for (int j = 0; j < (int) end_array_size; j++) {
			if (start_array[i].xHandle == end_array[j].xHandle) {
				k = j;
				//Mark that task have been matched by overwriting their handles
				start_array[i].xHandle = NULL;
				end_array[j].xHandle = NULL;
				break;
			}
		}
		//Check if matching task found
		if (k >= 0) {
			uint32_t task_elapsed_time = end_array[k].ulRunTimeCounter
				- start_array[i].ulRunTimeCounter;
			double percentage_time = (double)task_elapsed_time / (double)total_elapsed_time;
			percentage_time *= 100;
			char buffer[64];
//			uartStream.addInfo(std::to_string(percentage_time));
			sprintf(buffer, "| %-12s | %8lu | %0.1f%%\n", start_array[i].pcTaskName,
				task_elapsed_time, percentage_time);
			uartStream.addInfo(buffer);
		}
	}

//Print unmatched tasks
	for (int i = 0; i < (int) start_array_size; i++) {
		if (start_array[i].xHandle != NULL) {
			char buffer[64];
			sprintf(buffer, "| %s | Deleted\n", start_array[i].pcTaskName);
			uartStream.addInfo(buffer);
		}
	}
	for (int i = 0; i < (int) end_array_size; i++) {
		if (end_array[i].xHandle != NULL) {
			char buffer[64];
			sprintf(buffer, "| %s | Created\n", end_array[i].pcTaskName);
			uartStream.addInfo(buffer);
		}
	}
	ret = STM32_OK;

	free(start_array);
	free(end_array);
	return ret;
}
