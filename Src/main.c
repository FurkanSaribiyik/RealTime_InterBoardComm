/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include "GPIO_driver.h"
#include "RealtimeLib.h"

uint8_t TxBUFFER[]="I2C TX FUNCTIONALITY";
uint8_t RxBUFFER[128];
uint8_t request_datasize=128;
uint8_t targetaddr=0x51;
const uint8_t COMM_REQLENGTH=0x17;
const uint8_t COMM_REQDATA=0x18;
GPIO_Handle_t Handle;
I2C_Handle_t I2C_Handle;
uint8_t RxCmplt=RESET;
uint8_t TxCmplt=RESET;
uint8_t Handshake=RESET;
int main(void)
{
	//init_LEDs();
	init_scheduler_stack(SCHED_STACK_START);
	//init_task_stacks();
	switch_to_PSP();
	//init_systick_timer(TICK_HZ);

	tic();
	Handle.GPIOx=GPIOA;
	Handle.Config.GPIO_PinNumber=0;
	Handle.Config.GPIO_Mode=GPIO_MODE_INPUT;
	Handle.Config.GPIO_OutputSpeed=GPIO_OSPEED_HIGH;
	Handle.Config.GPIO_PUPD=GPIO_PUPD_PD;
	GPIO_Init(&Handle);

	I2C_Handle.I2Cx=I2C1;
	I2C_Handle.Config.I2C_ACKControl=I2C_ACK_ENABLE;
	I2C_Handle.Config.I2C_DeviceAddress=0x17;
	I2C_Handle.Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;
	I2C_Handle.Config.I2C_SCL_SPEED=I2C_SCL_SPEED_FM2K;

	I2C1_InterruptEnable();
	I2C1_InterruptPriority(1);
	I2C_Init(&I2C_Handle);
	I2C1_GPIOInit();
	unsigned int time=toc();
	printf("time elapsed in ms : %d \n",time);
	//volatile uint8_t dummy=0;
	while(1)
	{
		time=toc();
		printf("time elapsed in ms : %d \n",time/1000);
	}
/*
		if(Handshake==RESET)
		{
			while(Handshake==RESET)
			{
				while(I2C_master_senddata_IT(&I2C_Handle,(uint8_t *)(&COMM_REQLENGTH),1, targetaddr,I2C_REP_START_DISABLE)!=I2C_READY);
				while(TxCmplt!=SET)
				{
					if(dummy==20 && TxCmplt!=SET)
							{
								printf("TX FAILED \n");
								break;
							}
					printf("Trying to set TxCmplt %d\n",dummy);
					dummy++;
				}

				if(TxCmplt==SET)
				{
					Handshake=SET;
					printf("Handshake achieved with ESP32 requesting length\n");
					while(I2C_master_receivedata_IT(&I2C_Handle, &request_datasize,1, targetaddr,I2C_REP_START_DISABLE)!=I2C_READY);
					while(RxCmplt!=SET);
					printf("Length set %d \n", request_datasize);
				}
				TxCmplt=RESET;
			}
			dummy=0;
		}
		while(I2C_master_senddata_IT(&I2C_Handle,(uint8_t *)(&COMM_REQDATA),1, targetaddr,I2C_REP_START_DISABLE)!=I2C_READY);
		while(TxCmplt!=SET);


	while(1){
		while(!ReadFromInputPin(Handle.GPIOx, 0));
		for(int i=0;i<=200000;i++);
		while(I2C_master_senddata_IT(&I2C_Handle, TxBUFFER,strlen((char*)TxBUFFER), targetaddr,I2C_REP_START_DISABLE)!=I2C_READY);
		while(TxCmplt!=SET)
		{
			if(dummy==20 && TxCmplt!=SET)
					{
						printf("TX FAILED \n");
						break;
					}
			printf("Trying to set TxCmplt %d\n",dummy);
			dummy++;
		}
		dummy=0;
		if(TxCmplt==SET)
		{
			printf("DATA WAS SENT %s\n",TxBUFFER);
		}
		while(I2C_master_receivedata_IT(&I2C_Handle, RxBUFFER,request_datasize, targetaddr,I2C_REP_START_DISABLE)!=I2C_READY);
		while(RxCmplt!=SET);
		if(RxCmplt==SET && RxBUFFER[2]!=255)
		{
			printf("RxCMPLT: %d\n",RxCmplt);
			RxBUFFER[request_datasize]='\0';
			for(int i=0;i<=50000;i++);
			printf("Data Received: %s\n", RxBUFFER);
		}
		RxCmplt=RESET;
		TxCmplt=RESET;
		if(RxBUFFER[2]==255)
		{
			printf("255 FAILED 255 \n");
		}
		for(int i=0;i<128;i++)
		{
			RxBUFFER[i]=0;
		}

	}
*/
}

void I2C_AppEventCallback(I2C_Handle_t* pHandle,uint8_t Event)
{
	if(Event==I2C_EV_RX_CMPLT)
	{
		printf("RX COMPLETE\n");
		RxCmplt=SET;

	}else if(Event==I2C_EV_TX_CMPLT)
	{
		printf("TX COMPLETE\n");
		TxCmplt=SET;

	}else if(Event==I2C_ERROR_AF)
	{
		printf("ACK FAILED\n");
		if(pHandle->TxRxState==I2C_BUSY_RX)
		{
			I2C_CloseReceiveData(pHandle);
			I2C_GenerateStopCondition(pHandle->I2Cx);
		}else if(pHandle->TxRxState==I2C_BUSY_TX)
		{
			I2C_CloseSendData(pHandle);
			I2C_GenerateStopCondition(pHandle->I2Cx);
		}
	}else if(Event==I2C_ERROR_ARLO)
	{
		printf("ARBITRATION LOSS\n");
		if(pHandle->TxRxState==I2C_BUSY_RX)
		{
			I2C_CloseReceiveData(pHandle);
			I2C_GenerateStopCondition(pHandle->I2Cx);
		}else if(pHandle->TxRxState==I2C_BUSY_TX)
		{
			I2C_CloseSendData(pHandle);
			I2C_GenerateStopCondition(pHandle->I2Cx);
		}
	}else if(Event==I2C_ERROR_BERR)
	{
		printf("BUS ERROR\n");
		if(pHandle->TxRxState==I2C_BUSY_RX)
		{
			I2C_CloseReceiveData(pHandle);
			I2C_GenerateStopCondition(pHandle->I2Cx);
		}else if(pHandle->TxRxState==I2C_BUSY_TX)
		{
			I2C_CloseSendData(pHandle);
			I2C_GenerateStopCondition(pHandle->I2Cx);
		}
	}else if(Event==I2C_ERROR_OVR)
	{
		printf("OVERRUN ERROR\n");
		if(pHandle->TxRxState==I2C_BUSY_RX)
		{
			I2C_CloseReceiveData(pHandle);
			I2C_GenerateStopCondition(pHandle->I2Cx);
		}else if(pHandle->TxRxState==I2C_BUSY_TX)
		{
			I2C_CloseSendData(pHandle);
			I2C_GenerateStopCondition(pHandle->I2Cx);
		}
	}else if(Event==I2C_ERROR_TIMEOUT)
	{
		printf("TIMEOUT ERROR\n");
		if(pHandle->TxRxState==I2C_BUSY_RX)
		{
			I2C_CloseReceiveData(pHandle);
			I2C_GenerateStopCondition(pHandle->I2Cx);
		}else if(pHandle->TxRxState==I2C_BUSY_TX)
		{
			I2C_CloseSendData(pHandle);
			I2C_GenerateStopCondition(pHandle->I2Cx);
		}
	}
}

void task1(void)
{
	while(1)
	{
		turn_on_LED(GREEN_LED);
		task_delay(500);
		turn_off_LED(GREEN_LED);
		task_delay(500);
	}
}
void task2(void)
{
	while(1)
	{
		turn_on_LED(ORANGE_LED);
		task_delay(1000);
		turn_off_LED(ORANGE_LED);
		task_delay(1000);
	}
}

void task3(void)
{
	while(1)
	{
		turn_on_LED(RED_LED);
		task_delay(2000);
		turn_off_LED(RED_LED);
		task_delay(2000);
	}
}

void task4(void)
{
	while(1)
	{
		turn_on_LED(BLUE_LED);
		task_delay(250);
		turn_off_LED(BLUE_LED);
		task_delay(250);
	}
}




void	I2C1_EV_IRQHandler()
{
	I2C_EV_IRQHandling (&I2C_Handle);
}
void	I2C1_ER_IRQHandler()
{
	I2C_ER_IRQHandling (&I2C_Handle);
}


