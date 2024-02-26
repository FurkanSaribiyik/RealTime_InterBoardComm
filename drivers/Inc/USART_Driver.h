#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include <GPIO_driver.h>




typedef struct
{
    __vo uint32_t SR;   /* USART status register,                   Address offset: 0x00 */
    __vo uint32_t DR;   /* USART data register,                     Address offset: 0x04 */
    __vo uint32_t BRR;  /* USART baud rate register,                Address offset: 0x08 */
    __vo uint32_t CR1;  /* USART control register 1,                Address offset: 0x0C */
    __vo uint32_t CR2;  /* USART control register 2,                Address offset: 0x10 */
    __vo uint32_t CR3;  /* USART control register 3,                Address offset: 0x14 */
    __vo uint32_t GTPR; /* USART guard time and prescaler register, Address offset: 0x18 */
}USART_RegDef_t;

/*
 * Configuration structure for U(S)ARTx peripheral
 */
typedef struct
{
    uint8_t  USART_Mode;
    uint32_t USART_Baud;
    uint8_t  USART_NoOfStopBits;
    uint8_t  USART_WordLength;
    uint8_t  USART_ParityControl;
    uint8_t  USART_HWFlowControl;
}USART_Config_t;

#define RCC_APB1_ENR (RCC_BASEADDR+0X40)
#define RCC_APB1_RESET (RCC_BASEADDR+0X20)

#define RCC_APB2_ENR (RCC_BASEADDR+0X44)
#define RCC_APB2_RESET (RCC_BASEADDR+0X24)



/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */


#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

/****************************** Misc***********************/
#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U

#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)
#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)

#define USART1          ((USART_RegDef_t*) USART1_BASEADDR)
#define USART2          ((USART_RegDef_t*) USART2_BASEADDR)
#define USART3          ((USART_RegDef_t*) USART3_BASEADDR)
#define UART4           ((USART_RegDef_t*) UART4_BASEADDR)
#define UART5           ((USART_RegDef_t*) UART5_BASEADDR)
#define USART6          ((USART_RegDef_t*) USART6_BASEADDR)

#define USART1_PCCK_EN (*(uint32_t*)(RCC_APB2_ENR)|=(1<<4))
#define USART2_PCCK_EN (*(uint32_t*)(RCC_APB1_ENR)|=(1<<17))
#define USART3_PCCK_EN (*(uint32_t*)(RCC_APB1_ENR)|=(1<<18))
#define UART4_PCCK_EN (*(uint32_t*)(RCC_APB1_ENR)|=(1<<19))
#define UART5_PCCK_EN (*(uint32_t*)(RCC_APB1_ENR)|=(1<<20))
#define USART6_PCCK_EN (*(uint32_t*)(RCC_APB2_ENR)|=(1<<5))

#define USART1_PCCK_DIS (*(uint32_t*)(RCC_APB2_RESET)&=~(1<<4))
#define USART2_PCCK_DIS (*(uint32_t*)(RCC_APB1_RESET)&=~(1<<17))
#define USART3_PCCK_DIS (*(uint32_t*)(RCC_APB1_RESET)&=~(1<<18))
#define UART4_PCCK_DIS (*(uint32_t*)(RCC_APB1_RESET)&=~(1<<19))
#define UART5_PCCK_DIS (*(uint32_t*)(RCC_APB1_RESET)&=~(1<<20))
#define USART6_PCCK_DIS (*(uint32_t*)(RCC_APB2_RESET)&=~(1<<5))

#define USART1_IRQ_NO 37
#define USART2_IRQ_NO 38
#define USART3_IRQ_NO 39
#define UART4_IRQ_NO 52
#define UART5_IRQ_NO 53
#define USART6_IRQ_NO 71

/*
 * Handle structure for  U(S)ARTx peripheral
 */
typedef struct
{
    USART_RegDef_t *pUSARTx;    /* This holds the base address of USARTx(x:0,1,2) peripheral */
    USART_Config_t USART_Config;
    uint8_t *pTxBuffer;         /* Stores application Tx buffer address                      */
    uint8_t *pRxBuffer;         /* Stores application Rx buffer address                      */
    uint32_t TxLen;             /* Stores Tx length                                          */
    uint32_t RxLen;             /* Stores Rx length                                          */
    uint8_t TxBusyState;        /* Transmission is in  busy state                            */
    uint8_t RxBusyState;        /* Receiving is in  busy state                               */
}USART_Handle_t;

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx Register Addresses
 */
#define NVIC_ISER0      ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1      ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2      ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3      ( (__vo uint32_t*)0xE000E10C )
#define NVIC_ISER4      ( (__vo uint32_t*)0xE000E110 )
#define NVIC_ISER5      ( (__vo uint32_t*)0xE000E114 )
#define NVIC_ISER6      ( (__vo uint32_t*)0xE000E118 )
#define NVIC_ISER7      ( (__vo uint32_t*)0xE000E11C )


/*
 * ARM Cortex Mx Processor NVIC ICERx Register Addresses
 */
#define NVIC_ICER0      ( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1      ( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2      ( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3      ( (__vo uint32_t*)0XE000E18C )
#define NVIC_ICER4      ( (__vo uint32_t*)0XE000E190 )
#define NVIC_ICER5      ( (__vo uint32_t*)0XE000E194 )
#define NVIC_ICER6      ( (__vo uint32_t*)0XE000E198 )
#define NVIC_ICER7      ( (__vo uint32_t*)0XE000E19C )



/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR   ( (__vo uint32_t*)0XE000E400 )


/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * Peripheral definitions (Peripheral base addresses type casted to xxx_RegDef_t)
 */
/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX      0
#define USART_MODE_ONLY_RX      1
#define USART_MODE_TXRX         2


/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200         1200
#define USART_STD_BAUD_2400         2400
#define USART_STD_BAUD_9600         9600
#define USART_STD_BAUD_19200        19200
#define USART_STD_BAUD_38400        38400
#define USART_STD_BAUD_57600        57600
#define USART_STD_BAUD_115200       115200
#define USART_STD_BAUD_230400       230400
#define USART_STD_BAUD_460800       460800
#define USART_STD_BAUD_921600       921600
#define USART_STD_BAUD_2M           2000000
#define SUART_STD_BAUD_3M           3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_DISABLE        0
#define USART_PARITY_EN_EVEN        1
#define USART_PARITY_EN_ODD         2


/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS         0
#define USART_WORDLEN_9BITS         1


/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1        0
#define USART_STOPBITS_0_5      1
#define USART_STOPBITS_2        2
#define USART_STOPBITS_1_5      3


/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE         0
#define USART_HW_FLOW_CTRL_CTS          1
#define USART_HW_FLOW_CTRL_RTS          2
#define USART_HW_FLOW_CTRL_CTS_RTS      3


/*
 * USART Flags
 */
#define USART_FLAG_TXE      ( 1 << USART_SR_TXE  )
#define USART_FLAG_RXNE     ( 1 << USART_SR_RXNE )
#define USART_FLAG_TC       ( 1 << USART_SR_TC   )


/*
 * Application states
 */
#define USART_READY         0
#define USART_BUSY_IN_RX    1
#define USART_BUSY_IN_TX    2


/* Application events */
#define USART_EVENT_TX_CMPLT    0
#define	USART_EVENT_RX_CMPLT    1
#define	USART_EVENT_IDLE        2
#define	USART_EVENT_CTS         3
#define	USART_EVENT_PE          4
#define	USART_ERR_FE            5
#define	USART_ERR_NE            6
#define	USART_ERR_ORE           7


/******************************************************************************************
 *				APIs supported by this driver
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
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Length);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Length);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Length);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Length);


/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);


/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint8_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t FlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEvent);


#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
