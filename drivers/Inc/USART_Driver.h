/*
 * USART_API_prototype.h
 *
 *  Created on: Feb 16, 2024
 *      Author: Furkan
 */

#ifndef INC_USART_DRIVER_H_
#define INC_USART_DRIVER_H_

#include "GPIO_driver.h"

#define APB1_CLK HSI_CLK
#define APB2_CLK HSI_CLK

#define NO_PR_BITS_IMPLEMENTED  4

#define NVIC_PR_BASE_ADDR 	((uint32_t*)0xE000E400)

#define NVIC_ISER0          ((uint32_t*)0xE000E100 )
#define NVIC_ISER1          ((uint32_t*)0xE000E104 )
#define NVIC_ISER2          ((uint32_t*)0xE000E108 )
#define NVIC_ISER3          ((uint32_t*)0xE000E10C )

#define NVIC_ICER0 			((uint32_t*)0XE000E180)
#define NVIC_ICER1			((uint32_t*)0XE000E184)
#define NVIC_ICER2  		((uint32_t*)0XE000E188)
#define NVIC_ICER3			((uint32_t*)0XE000E18C)

#define RCC_APB1_ENR (RCC_BASEADDR+0X40)
#define RCC_APB1_RESET (RCC_BASEADDR+0X20)

#define RCC_APB2_ENR (RCC_BASEADDR+0X44)
#define RCC_APB2_RESET (RCC_BASEADDR+0X24)

#define USART1_BASEADDR 0x40011000
#define USART2_BASEADDR 0x40004400
#define USART3_BASEADDR 0x40004800
#define UART4_BASEADDR 0x40004C00
#define UART5_BASEADDR 0x40005000
#define USART6_BASEADDR 0x40011400

#define USART1_PCLK_EN (*(uint32_t*)(RCC_APB2_ENR)|=(1<<4))
#define USART2_PCLK_EN (*(uint32_t*)(RCC_APB1_ENR)|=(1<<17))
#define USART3_PCLK_EN (*(uint32_t*)(RCC_APB1_ENR)|=(1<<18))
#define UART4_PCLK_EN (*(uint32_t*)(RCC_APB1_ENR)|=(1<<19))
#define UART5_PCLK_EN (*(uint32_t*)(RCC_APB1_ENR)|=(1<<20))
#define USART6_PCLK_EN (*(uint32_t*)(RCC_APB2_ENR)|=(1<<5))

#define USART1_PCLK_DIS (*(uint32_t*)(RCC_APB2_RESET)&=~(1<<4))
#define USART2_PCLK_DIS (*(uint32_t*)(RCC_APB1_RESET)&=~(1<<17))
#define USART3_PCLK_DIS (*(uint32_t*)(RCC_APB1_RESET)&=~(1<<18))
#define UART4_PCLK_DIS (*(uint32_t*)(RCC_APB1_RESET)&=~(1<<19))
#define UART5_PCLK_DIS (*(uint32_t*)(RCC_APB1_RESET)&=~(1<<20))
#define USART6_PCLK_DIS (*(uint32_t*)(RCC_APB2_RESET)&=~(1<<5))

#define USART_CR1_SBK 0
#define USART_CR1_RWU 1
#define USART_CR1_RE 2
#define USART_CR1_TE 3
#define USART_CR1_IDLEIE 4
#define USART_CR1_RXNEIE 5
#define USART_CR1_TCIE 6
#define USART_CR1_TXEIE 7
#define USART_CR1_PEIE 8
#define USART_CR1_PS 9
#define USART_CR1_PCE 10
#define USART_CR1_WAKE 11
#define USART_CR1_M 12
#define USART_CR1_UE 13
#define USART_CR1_OVER8 15

#define USART_CR2_STOP 12

#define USART_CR3_RTSE 8
#define USART_CR3_CTSE 9


#define USART_FLAG_RESET 0
#define USART_FLAG_SET 1

#define USART_SR_PE 0
#define USART_SR_FE 1
#define USART_SR_NF 2
#define USART_SR_ORE 3
#define USART_SR_IDLE 4
#define USART_SR_RXNE 5
#define USART_SR_TC 6
#define USART_SR_TXE 7
#define USART_SR_LBD 8
#define USART_SR_CTS 9

typedef struct
{
	uint32_t USART_SR;
	uint32_t USART_DR;
	uint32_t USART_BRR;
	uint32_t USART_CR1;
	uint32_t USART_CR2;
	uint32_t USART_CR3;
	uint32_t USART_GTPR;
}USART_RegDef_t;

#define USART1 (USART_RegDef_t *)(USART1_BASEADDR)
#define USART2 (USART_RegDef_t *)(USART2_BASEADDR)
#define USART3 (USART_RegDef_t *)(USART3_BASEADDR)
#define UART4 (USART_RegDef_t *)(UART4_BASEADDR)
#define UART5 (USART_RegDef_t *)(UART5_BASEADDR)
#define USART6 (USART_RegDef_t *)(USART6_BASEADDR)


/*
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;


/*
 * Handle structure for USARTx peripheral
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t   USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;


/*
 * Application states
 */
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3



/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void  USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);

#endif /* INC_USART_DRIVER_H_ */
