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

typedef struct{
  int16_t a_x;
  int16_t a_y;
  int16_t a_z;

  int16_t g_x;
  int16_t g_y;
  int16_t g_z;

  int16_t temp;
  int16_t padding;
  int longtitude;
  int latitude;
}readData;
readData ESP1_Datas;
readData ESP2_Datas;


uint8_t task_needs_stay=RESET;
//reply from arduino will be stored here
char rx_buf[128] ;
char USART_handshake_msg[]="handshake";
char USART_send_clearBuffer[sizeof(USART_handshake_msg)-1];

uint8_t USART_HandshakeCmplt=RESET;
USART_Handle_t usart2_handle;
uint8_t rxCmplt = RESET;
uint8_t txCmplt = RESET;




uint8_t TxBUFFER_ESP_1[]="I2C TX FUNCTIONALITY";
uint8_t RxBUFFER_ESP_1[128];
uint8_t SIZE_data_1=128;
uint8_t ADDR_I2C_ESP_1=0x51;
uint8_t RxCmplt_ESP_1=RESET;
uint8_t TxCmplt_ESP_1=RESET;
uint8_t Handshake_ESP_1=RESET;

uint8_t SIZE_data_2=sizeof(ESP2_Datas);
uint8_t ADDR_I2C_ESP_2=0x52;
uint8_t RxCmplt_ESP_2=RESET;
uint8_t TxCmplt_ESP_2=RESET;
uint8_t Handshake_ESP_2=RESET;


const uint8_t COMM_REQLENGTH=0x17;
const uint8_t COMM_REQDATA=0x18;
GPIO_Handle_t Handle;
I2C_Handle_t I2C_Handle;

void USART2_Init(void);

void USART2_GPIOInit(void);

void GPIO_ButtonInit(void);

void blocking_delay(void);

void USART_Handshake(void);

void I2C_ESP1_Handshake_ESP_1(void);
void READI2C_ESP1(void);

void I2C_ESP2_Handshake_ESP_2(void);
void READI2C_ESP2(void);

int main(void)
{

	GPIO_ButtonInit();
	init_scheduler_stack(SCHED_STACK_START);
	I2C_Handle.I2Cx=I2C1;
	I2C_Handle.Config.I2C_ACKControl=I2C_ACK_ENABLE;
	I2C_Handle.Config.I2C_DeviceAddress=0x17;
	I2C_Handle.Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;
	I2C_Handle.Config.I2C_SCL_SPEED=I2C_SCL_SPEED_SM;

	I2C1_InterruptEnable();
	I2C1_InterruptPriority(2);
	I2C_Init(&I2C_Handle);
	I2C1_GPIOInit();

	USART2_GPIOInit();
    USART2_Init();
    USART_IRQInterruptConfig(USART2_IRQ_NO,ENABLE);
    USART_IRQPriorityConfig(USART2_IRQ_NO,1);
    USART_PeripheralControl(USART2,ENABLE);

    turn_on_LED(GREEN_LED);		//WAIT UNTIL GREEN LIGHT TO CONNECT THE TX/RX LINE
    printf("PLEASE CONNECT TXRX LINE TO ARDUINO\n");
	while( ! ReadFromInputPin(GPIOA,0) );	//Press the button to continue
	blocking_delay();
	turn_off_LED(GREEN_LED);
	I2C_ESP1_Handshake_ESP_1();
	I2C_ESP2_Handshake_ESP_2();
    USART_Handshake();
    turn_on_LED(BLUE_LED);		//IF HANDSHAKE IS SUCCESSFULL BLUE LIGHT WILL TURN ON
    while( ! ReadFromInputPin(GPIOA,0) );	//Press the button to continue
    blocking_delay();
    printf("Handshake Successfull, please press the button to send DATA \n");
    turn_off_LED(BLUE_LED);

    READI2C_ESP1();
    READI2C_ESP2();

	init_task_stacks();
	switch_to_PSP();
	init_systick_timer(TICK_HZ);
	task1();
}

void I2C_AppEventCallback(I2C_Handle_t* pHandle,uint8_t Event)
{
	if(Event==I2C_EV_RX_CMPLT)
	{
		//printf("RX COMPLETE\n");
		if(pHandle->SlaveAddr==0x51)
		RxCmplt_ESP_1=SET;
		else
		RxCmplt_ESP_2=SET;

	}else if(Event==I2C_EV_TX_CMPLT)
	{
		//printf("TX COMPLETE\n");
		if(pHandle->SlaveAddr==0x51)
		TxCmplt_ESP_1=SET;
		else
		TxCmplt_ESP_2=SET;

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
		task_needs_stay=SET;
		READI2C_ESP1();
		READI2C_ESP2();
		task_needs_stay=RESET;
		task_delay(30);
	}
}
void task2(void)
{
	while(1)
	{
		task_needs_stay=SET;
		USART_SendDataIT(&usart2_handle,(uint8_t*)&ESP1_Datas,sizeof(ESP1_Datas));
		while(txCmplt!=SET);
		txCmplt=RESET;
		USART_SendDataIT(&usart2_handle,(uint8_t*)&ESP2_Datas,sizeof(ESP2_Datas));
		while(txCmplt!=SET);
		txCmplt=RESET;
		task_needs_stay=RESET;
		task_delay(30);
	}
}
void I2C_ESP1_Handshake_ESP_1(void)
{
	volatile uint8_t dummy=0;

		if(Handshake_ESP_1==RESET)
		{
			while(Handshake_ESP_1==RESET)
			{
				while(I2C_master_senddata_IT(&I2C_Handle,(uint8_t *)(&COMM_REQLENGTH),1, ADDR_I2C_ESP_1,I2C_REP_START_DISABLE)!=I2C_READY);
				while(TxCmplt_ESP_1!=SET)
				{
					if(dummy==20 && TxCmplt_ESP_1!=SET)
							{
								printf("TX FAILED \n");
								break;
							}
					printf("Trying to set TxCmplt_ESP_1 %d\n",dummy);
					dummy++;
				}

				if(TxCmplt_ESP_1==SET)
				{
					Handshake_ESP_1=SET;
					printf("Handshake_ESP_1 achieved with ESP32 requesting length\n");
					while(I2C_master_receivedata_IT(&I2C_Handle, &SIZE_data_1,1, ADDR_I2C_ESP_1,I2C_REP_START_DISABLE)!=I2C_READY);
					while(RxCmplt_ESP_1!=SET);
					printf("Length set %d \n", SIZE_data_1);
				}
				TxCmplt_ESP_1=RESET;
			}
			dummy=0;
		}
		while(I2C_master_senddata_IT(&I2C_Handle,(uint8_t *)(&COMM_REQDATA),1, ADDR_I2C_ESP_1,I2C_REP_START_DISABLE)!=I2C_READY);
		while(TxCmplt_ESP_1!=SET);
		TxCmplt_ESP_1=RESET;
}

void READI2C_ESP1(void)
{
			while(I2C_master_senddata_IT(&I2C_Handle, (uint8_t *)&COMM_REQDATA,SIZE_data_1, ADDR_I2C_ESP_1,I2C_REP_START_ENABLE)!=I2C_READY);
			while(TxCmplt_ESP_1!=SET);
			while(I2C_master_receivedata_IT(&I2C_Handle, (uint8_t*)&ESP1_Datas,SIZE_data_1, ADDR_I2C_ESP_1,I2C_REP_START_DISABLE)!=I2C_READY);
			while(RxCmplt_ESP_1!=SET);
			if(RxCmplt_ESP_1==SET)
			{
				/*
				printf("RxCmplt_ESP_2: %d\n",RxCmplt_ESP_1);
				printf("ESP1 : AX %d, AY %d, AZ %d\n",ESP1_Datas.a_x, ESP1_Datas.a_y, ESP1_Datas.a_z);
				printf(" GX %d, GY %d, GZ %d, TEMP %d, LAT %d, LONG %d \n", ESP1_Datas.g_x, ESP1_Datas.g_y, ESP1_Datas.g_z, ESP1_Datas.temp, ESP1_Datas.latitude, ESP1_Datas.longtitude);
				*/
			}
			RxCmplt_ESP_1=RESET;
			TxCmplt_ESP_1=RESET;
			if(ESP1_Datas.longtitude<0 || ESP1_Datas.latitude<0 || ESP1_Datas.temp<0)
			{
				printf("WTF, WTF , WTF ,WTF ,WTF ,WTF,WTF ,WTF");
			}
}




void I2C_ESP2_Handshake_ESP_2(void)
{
	volatile uint8_t dummy=0;

		if(Handshake_ESP_2==RESET)
		{
			while(Handshake_ESP_2==RESET)
			{
				while(I2C_master_senddata_IT(&I2C_Handle,(uint8_t *)(&COMM_REQLENGTH),1, ADDR_I2C_ESP_2,I2C_REP_START_DISABLE)!=I2C_READY);
				while(TxCmplt_ESP_2!=SET)
				{
					if(dummy==20 && TxCmplt_ESP_2!=SET)
							{
								printf("TX FAILED \n");
								break;
							}
					printf("Trying to set TxCmplt_ESP_2 %d\n",dummy);
					dummy++;
				}

				if(TxCmplt_ESP_2==SET)
				{
					Handshake_ESP_2=SET;
					printf("Handshake_ESP_2 achieved with ESP32 requesting length\n");
					while(I2C_master_receivedata_IT(&I2C_Handle, &SIZE_data_2,1, ADDR_I2C_ESP_2,I2C_REP_START_DISABLE)!=I2C_READY);
					while(RxCmplt_ESP_2!=SET);
					printf("Length set %d \n", SIZE_data_2);
				}
				TxCmplt_ESP_2=RESET;
			}
			dummy=0;
		}
		while(I2C_master_senddata_IT(&I2C_Handle,(uint8_t *)(&COMM_REQDATA),1, ADDR_I2C_ESP_2,I2C_REP_START_DISABLE)!=I2C_READY);
		while(TxCmplt_ESP_2!=SET);
		TxCmplt_ESP_2=RESET;
}

void READI2C_ESP2(void)
{
			while(I2C_master_senddata_IT(&I2C_Handle, (uint8_t *)&COMM_REQDATA,SIZE_data_2, ADDR_I2C_ESP_2,I2C_REP_START_ENABLE)!=I2C_READY);
			while(TxCmplt_ESP_2!=SET);
			while(I2C_master_receivedata_IT(&I2C_Handle, (uint8_t*)&ESP2_Datas,SIZE_data_2, ADDR_I2C_ESP_2,I2C_REP_START_DISABLE)!=I2C_READY);
			while(RxCmplt_ESP_2!=SET);
			if(RxCmplt_ESP_2==SET)
			{
				/*
				printf("RxCmplt_ESP_2: %d\n",RxCmplt_ESP_2);
				printf("ESP1 : AX %d, AY %d, AZ %d\n",ESP2_Datas.a_x, ESP2_Datas.a_y, ESP2_Datas.a_z);
				printf(" GX %d, GY %d, GZ %d, TEMP %d, LAT %d, LONG %d \n", ESP2_Datas.g_x, ESP2_Datas.g_y, ESP2_Datas.g_z, ESP2_Datas.temp, ESP2_Datas.latitude, ESP2_Datas.longtitude);
				*/
			}
			RxCmplt_ESP_2=RESET;
			if(ESP1_Datas.longtitude<0 || ESP1_Datas.latitude<0 || ESP1_Datas.temp<0)
			{
				printf("WTF, WTF , WTF ,WTF ,WTF ,WTF,WTF ,WTF");
			}
}

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}

void 	USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.GPIOx = GPIOA;
	usart_gpios.Config.GPIO_Mode = GPIO_MODE_ALT_FUNC;
	usart_gpios.Config.GPIO_OutputType = GPIO_OTYPE_PP;
	usart_gpios.Config.GPIO_PUPD = GPIO_PUPD_PU;
	usart_gpios.Config.GPIO_OutputSpeed = GPIO_OSPEED_VERY_HIGH;
	usart_gpios.Config.GPIO_AlternateFunc =GPIO_ALT_FUNC7;

	usart_gpios.Config.GPIO_PinNumber  = 2; //TX
	GPIO_Init(&usart_gpios);

	usart_gpios.Config.GPIO_PinNumber = 3; //RX
	GPIO_Init(&usart_gpios);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GPIOLED;

	//this is btn gpio configuration
	GPIOBtn.GPIOx = GPIOA;
	GPIOBtn.Config.GPIO_PinNumber = 0;
	GPIOBtn.Config.GPIO_Mode = GPIO_MODE_INPUT;
	GPIOBtn.Config.GPIO_OutputSpeed = GPIO_OSPEED_VERY_HIGH;
	GPIOBtn.Config.GPIO_PUPD = GPIO_PUPD_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	GPIOLED.GPIOx = GPIOD;
	GPIOLED.Config.GPIO_PinNumber = 15;
	GPIOLED.Config.GPIO_Mode = GPIO_MODE_OUTPUT;
	GPIOLED.Config.GPIO_OutputSpeed = GPIO_OSPEED_VERY_HIGH;
	GPIOLED.Config.GPIO_PUPD = GPIO_PUPD_PD;

	GPIO_Init(&GPIOLED);

	GPIOLED.Config.GPIO_PinNumber = 12;
	GPIO_Init(&GPIOLED);
	GPIOLED.Config.GPIO_PinNumber = 13;
	GPIO_Init(&GPIOLED);
	GPIOLED.Config.GPIO_PinNumber = 14;
	GPIO_Init(&GPIOLED);
}

void blocking_delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

void USART_Handshake(void)
{
	while(USART_HandshakeCmplt!=SET)
	{
		while(USART_ReceiveDataIT(&usart2_handle,(uint8_t *)rx_buf,strlen(USART_handshake_msg))!=USART_READY);

		while(USART_SendDataIT(&usart2_handle,(uint8_t*)USART_handshake_msg,strlen(USART_handshake_msg))!=USART_READY);
		//Send the msg indexed by cnt in blocking mode

		while(txCmplt != SET);
		while(rxCmplt != SET);
		if(!strcmp(rx_buf,USART_handshake_msg))
		{
			printf("USART_HANDSHAKE SUCCESS\n");
			USART_HandshakeCmplt=SET;

		}else
		{
			printf("USART_HANDSHAKE FAILED\n");
		}
		txCmplt=RESET;
		rxCmplt=RESET;
	}
}

void USART2_IRQHandler(void)
{
	USART_IRQHandling(&usart2_handle);
}





void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
   if(ApEv == USART_EVENT_RX_CMPLT)
   {
			rxCmplt = SET;

   }else if (ApEv == USART_EVENT_TX_CMPLT)
   {
	   txCmplt= SET;
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
