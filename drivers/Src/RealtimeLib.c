/*
 * RealtimeLib.c
 *
 *  Created on: Feb 16, 2024
 *      Author: Furkan
 */


#ifndef INC_REALTIMELIB_C_
#define INC_REALTIMELIB_C_
#include "RealtimeLib.h"




TCB_t user_tasks[NUMBEROF_TASKS];
uint32_t g_tick_count=0;
uint8_t current_task=1;
uint32_t timer_now=0;

void init_LEDs(void)
{
	volatile uint32_t* const pClockEnReg= (uint32_t *) ((0x40023800)+(0x30));
	volatile uint32_t* const pGpio_D_ModeReg= (uint32_t *) (0x40020C00);
	*pClockEnReg|=(1<<3);	//Enabling gpioD clock
	*pGpio_D_ModeReg&=~(3<<24); //Clearing the bits for GREEN LED
	*pGpio_D_ModeReg&=~(3<<26); //Clearing the bits for ORANGE LED
	*pGpio_D_ModeReg&=~(3<<28); //Clearing the bits for RED LED
	*pGpio_D_ModeReg&=~(3<<30); //Clearing the bits for BLUE LED
	*pGpio_D_ModeReg|= (1<<24);	//Setting bit 24 (PD12) for output GREEN LED
	*pGpio_D_ModeReg|= (1<<26);	//Setting bit 26 (PD13) for output ORANGE LED
	*pGpio_D_ModeReg|= (1<<28);	//Setting bit 28 (PD14)for output RED LED
	*pGpio_D_ModeReg|= (1<<30);	//Setting bit 30 (PD15)for output BLUE LED
}

void turn_on_LED(uint8_t led)
{
	volatile uint32_t* const pGpio_D_OutputReg= (uint32_t *) ((0x40020C00)+(0x14));
	*pGpio_D_OutputReg|=(1<<(12+led));
}
void turn_off_LED(uint8_t led)
{
	volatile uint32_t* const pGpio_D_OutputReg= (uint32_t *) ((0x40020C00)+(0x14));
	*pGpio_D_OutputReg&=~(1<<(12+led));
}


void set_next_task(void)
{
	if(task_needs_stay==0)
	{
	int state=TASK_BLOCKED_STATE;
		for(int i=0;i<(NUMBEROF_TASKS);i++)
		{
			current_task++;
			current_task%=NUMBEROF_TASKS;
			state=user_tasks[current_task].current_state;
			if((state==TASK_READY_STATE)&&(current_task!=0))
			{
				break;
			}
		}
		if(state!=TASK_READY_STATE)
		{
			current_task=0;
		}

	}
}



void update_global_tick_count(void)
{
	volatile uint32_t val=(uint32_t)0xDFFFFFFFF;
	if(g_tick_count==val)
	{
		volatile uint32_t subt=(uint32_t)0xBFFFFFFFF;
		g_tick_count-=subt;

		for(int i=1;i<NUMBEROF_TASKS;i++)
		{
			user_tasks[i].block_count-=subt;
		}

		g_tick_count++;
	}
	else
	{
		g_tick_count++;
	}
}

void unblock_tasks(void)
{
	for(int i=1;i<NUMBEROF_TASKS;i++)
	{
		if(user_tasks[i].current_state!=TASK_READY_STATE)
		{
			if(user_tasks[i].block_count<=g_tick_count)
			{
				user_tasks[i].current_state=TASK_READY_STATE;
			}
		}
	}
}

void SysTick_Handler(void)
{
	update_global_tick_count();
	unblock_tasks();
	schedule_pendsv();
}

void tic(void)
{
	timer_now=g_tick_count;
}
uint32_t toc(void)
{
	return (g_tick_count-timer_now);
}

__attribute((naked)) void PendSV_Handler(void)
{
	__asm volatile ("MRS R0,PSP");		//saving the current value of the PSP
	__asm volatile ("STMDB R0!,{R4-R11}");	//saving the remaining piece of the stack frame

	__asm volatile ("PUSH {LR}");		//Save the return value in order to go back and forth between functions
	__asm volatile ("BL save_psp_value");	//Saving the location of PSP

	__asm volatile ("BL set_next_task");

	__asm volatile ("BL get_psp_value");	//get the psp of the next task

	__asm volatile ("LDMIA R0!,{R4-R11}");	//recover the bottom of the stack frame of the next task, the rest of the frame will be recovered at the end of the interrupt

	__asm volatile ("MSR PSP,R0");

	__asm volatile ("POP {LR}");

	__asm volatile ("BX LR");		//return to main
}

uint32_t get_psp_value(void)
{
	return user_tasks[current_task].psp_val;
}
void save_psp_value(uint32_t psp_addr)
{
	user_tasks[current_task].psp_val=psp_addr;
}

__attribute((naked)) void switch_to_PSP(void)
{

	__asm volatile ("PUSH {LR}");			//Save the return value to jump to another function
	__asm volatile ("BL get_psp_value");	//Get PSP of the current stack and store it in R0
	__asm volatile ("POP {LR}");
	__asm volatile ("MSR PSP,R0");			//PSP value is loaded into PSP register


	/*
	__asm volatile (".equ SRAM_END, (0x20000000+(128*1024))");
	__asm volatile ("LDR R0,=SRAM_END");
	__asm volatile ("MSR PSP,R0");			//Setting PSP to the top of the stack
	*/

	__asm volatile ("MOV R0,#0X02");
	__asm volatile ("MSR CONTROL,R0");		//Set main SP as PSP
	__asm volatile ("BX LR");		//Return to main
}


void init_task_stacks(void)
{

	user_tasks[0].current_state=TASK_READY_STATE;
	for(int i=1;i<NUMBEROF_TASKS;i++)
	{
		user_tasks[i].current_state=TASK_READY_STATE;
	}

	user_tasks[0].psp_val=IDLE_STACK_START;
	for(int i=1;i<NUMBEROF_TASKS;i++)
	{
		user_tasks[i].psp_val=(T1_STACK_START-((i-1)*SIZE_TASK_STACK));
	}

	user_tasks[0].task_handler=idle_task;
	user_tasks[1].task_handler=task1;
	user_tasks[2].task_handler=task2;
	//user_tasks[3].task_handler=task3;
	//user_tasks[4].task_handler=task4;

	uint32_t *pPSP;
	for(int i=0;i<NUMBEROF_TASKS;i++)		//the stackframe of task1 will be loaded when the systick interrupt triggers
	{
		pPSP=(uint32_t *)user_tasks[i].psp_val;
		--pPSP;
		*pPSP=0x01000000;	//Setting xPSR register with reset value, T bit is set as 1
		--pPSP;
		*pPSP= (uint32_t)user_tasks[i].task_handler;		//Address of the function/tasks
		--pPSP;
		*pPSP=0xFFFFFFFD;	//Loading LR with exception return to handler mode that returns with PSP as SP
		for(int k=0;k<13;k++)
		{
			--pPSP;
			*pPSP=0;
		}
		user_tasks[i].psp_val=(uint32_t)pPSP;			//
	}
}


__attribute((naked)) void init_scheduler_stack (uint32_t scheduleraddr)
{
	__asm volatile ("MSR MSP,R0");
	__asm volatile ("BX LR");
}

void init_systick_timer(uint32_t tickHz)
{
	uint32_t counter=(SYSTICK_CLK/tickHz)-1;		//calculating how many ticks are necessary for the requested time frequency
	uint32_t* Systick_Control_Reg=(uint32_t *)0xE000E010;
	uint32_t* Systick_Counter_Reg=(uint32_t *)0xE000E014;

	*Systick_Control_Reg|=(7<<0);				/* Enabling counter register countdown, Systick exception and choosing internal processor clock 16Mhz*/

	*Systick_Counter_Reg&=~(0x00FFFFFF);	//clearing Counter Register
	*Systick_Counter_Reg|=(counter<<0);

}


void schedule_pendsv(void)
{
	uint32_t *pICSR=(uint32_t*)PENDSV_REG;
	*pICSR|=(1<<28);
}

void task_delay(uint32_t tick_count)
{
	if(current_task)
	{
	user_tasks[current_task].block_count= g_tick_count + tick_count;
	user_tasks[current_task].current_state= TASK_BLOCKED_STATE;
	schedule_pendsv();
	}
}



void idle_task(void)
{
	while(1)
	{

	}
}


#endif /* INC_REALTIMELIB_C_ */
