/*
 * RealtimeLib.h
 *
 *  Created on: Feb 16, 2024
 *      Author: Furkan
 */

#ifndef INC_REALTIMELIB_H_
#define INC_REALTIMELIB_H_

#include <stdint.h>

#define SIZE_TASK_STACK 1024U
#define SRAM_START 0X20000000U
#define SRAM_SIZE ((128)*(1024))
#define SRAM_END ((SRAM_START)+(SRAM_SIZE))

#define T1_STACK_START SRAM_END
#define T2_STACK_START ((SRAM_END) -(1*SIZE_TASK_STACK))
//#define T3_STACK_START ((SRAM_END) -(2*SIZE_TASK_STACK))
//#define T4_STACK_START ((SRAM_END) -(3*SIZE_TASK_STACK))
#define IDLE_STACK_START ((SRAM_END) -(4*SIZE_TASK_STACK))
#define SCHED_STACK_START ((SRAM_END) -(5*SIZE_TASK_STACK))

#define TICK_HZ	1000U
#define CPU_CLK 16000000U
#define SYSTICK_CLK CPU_CLK

#define TASK_READY_STATE 0X00
#define TASK_BLOCKED_STATE 0XFF
#define NUMBEROF_TASKS 3

#define PENDSV_REG 0xE000ED04U

#define GREEN_LED 0
#define ORANGE_LED 1
#define RED_LED 2
#define BLUE_LED 3

void task1(void);
void task2(void);
void task3(void);
void task4(void);

void init_systick_timer(uint32_t tickHz);
__attribute((naked)) void init_scheduler_stack(uint32_t scheduleraddr);
__attribute((naked)) void switch_to_PSP(void);
void init_task_stacks(void);
uint32_t get_psp_value(void);
void task_delay(uint32_t tick_count);
void idle_task(void);
void update_global_tick_count(void);
void schedule_pendsv(void);
void init_LEDs(void);
void turn_on_LED(uint8_t led);
void turn_off_LED(uint8_t led);

void tic(void);
uint32_t toc(void);

extern uint32_t g_tick_count;
extern uint8_t current_task;
extern uint32_t timer_now;
extern uint8_t task_needs_stay;

typedef struct {
	uint32_t psp_val;
	uint32_t block_count;
	uint8_t current_state;
	void (*task_handler)(void);
}TCB_t;


#endif /* INC_REALTIMELIB_H_ */
