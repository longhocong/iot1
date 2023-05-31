/*
 * scheduler.c
 *
 *  Created on: Mar 25, 2023
 *      Author: LOC
 */
#include "scheduler.h"
sTasks SCH_tasks_G[SCH_MAX_TASKS];
uint8_t current_index_task=0;

void SCH_Init(void){
	current_index_task =0;
}

void SCH_Add_Task(void(*pFunction)(),
					uint32_t DELAY,
					uint32_t PERIOD){
	if (current_index_task < SCH_MAX_TASKS){
		SCH_tasks_G[current_index_task].pTask=pFunction;
		SCH_tasks_G[current_index_task].Delay= DELAY;
		SCH_tasks_G[current_index_task].Period= PERIOD;
		SCH_tasks_G[current_index_task].Runme= 0;
		SCH_tasks_G[current_index_task].TaskID= current_index_task;
		current_index_task++;
	}}
void SCH_Update(void){
	for(int i=0;i<current_index_task; i++){
		if(SCH_tasks_G[i].Delay>0){
			SCH_tasks_G[i].Delay--;
		}else{
			SCH_tasks_G[i].Delay=SCH_tasks_G[i].Period;
			SCH_tasks_G[i].Runme +=1;
		}
	}
}
void SCH_Dispatch_Tasks(void){
	for(int i=0; i<current_index_task; i++){
		if(SCH_tasks_G[i].Runme >0){
			SCH_tasks_G[i].Runme--;
			(*SCH_tasks_G[i].pTask)();
		}
	}
}
