/*
 * I2C_Driver.c
 *
 *  Created on: Jan 31, 2024
 *      Author: Furkan
 */


#ifndef I2C_DRIVER_C_
#define I2C_DRIVER_C_

#include "I2C_Driver.h"

static void I2C_GenerateStartCondition(I2C_Regdef_t* pI2Cx);
static uint8_t I2C_GetFlagStatus(I2C_Regdef_t* pI2Cx, uint32_t FlagName);
static void I2C_AddressPhase_Write(I2C_Regdef_t* pI2Cx,uint8_t SlaveAddr);
static void I2C_AddressPhase_Read(I2C_Regdef_t* pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_Regdef_t* pI2Cx);
static void I2C_HandleACK(I2C_Regdef_t* pI2Cx, uint8_t ENORDI);
static void I2C_TxEIRQHandler(I2C_Handle_t* pHandle);
static void I2C_RxNEIRQHandler(I2C_Handle_t* pHandle);



static void I2C_HandleACK(I2C_Regdef_t* pI2Cx, uint8_t ENORDI)
{
	if(ENORDI==ENABLE)
	{
		pI2Cx->I2C_CR1|=(1<<10);			//ENABLING ACK
	}else
	{
		pI2Cx->I2C_CR1&=~(1<<10);		//DISABLING ACK
	}

}

static void I2C_GenerateStartCondition(I2C_Regdef_t* pI2Cx)
{
	pI2Cx->I2C_CR1|=(1<<I2C_CR1_START);
}

static uint8_t I2C_GetFlagStatus(I2C_Regdef_t* pI2Cx, uint32_t FlagName)
{
	if(FlagName<=(1<<15))
	{
		if(pI2Cx->I2C_SR1&FlagName)
		{
			return I2C_FLAG_SET;
		}
	}else
	{
		FlagName=(FlagName>>16);
		if(pI2Cx->I2C_SR2&FlagName)
		{
			return I2C_FLAG_SET;
		}
	}
	return I2C_FLAG_RESET;
}

static void I2C_AddressPhase_Write(I2C_Regdef_t* pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr=(SlaveAddr<<1);
	SlaveAddr&=~(1);
	pI2Cx->I2C_DR=SlaveAddr;
}

static void I2C_AddressPhase_Read(I2C_Regdef_t* pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr=(SlaveAddr<<1);
	SlaveAddr|=(1);
	pI2Cx->I2C_DR=SlaveAddr;
}

static void I2C_ClearAddrFlag(I2C_Regdef_t* pI2Cx)
{
	uint32_t temp=pI2Cx->I2C_SR1;
	temp=pI2Cx->I2C_SR2;
	(void)temp;
}

void I2C_GenerateStopCondition(I2C_Regdef_t* pI2Cx)
{
	pI2Cx->I2C_CR1|=(1<<I2C_CR1_STOP);
}

static void I2C_TxEIRQHandler(I2C_Handle_t* pHandle)
{
	if(pHandle->TxLen>0)
	{
		pHandle->I2Cx->I2C_DR=*pHandle->pTxBuffer;
		pHandle->TxLen--;
		pHandle->pTxBuffer++;
	}
}


static void I2C_RxNEIRQHandler(I2C_Handle_t* pHandle)
{
	//handle RxNE condition
	if(I2C_GetFlagStatus(pHandle->I2Cx,I2C_SR2_FLAG_MSL))	//Check for master mode
	{if(pHandle->TxRxState==I2C_BUSY_RX)
		{if(pHandle->RxSize==1)
			{
				*pHandle->pRxBuffer=pHandle->I2Cx->I2C_DR;
				pHandle->pRxBuffer++;
				pHandle->RxLen--;
			}else if(pHandle->RxSize>1)
			{
				if(pHandle->RxLen==2)
				{
					//Disable ACKing
					I2C_HandleACK(pHandle->I2Cx,I2C_ACK_DISABLE);
				}
				*pHandle->pRxBuffer=pHandle->I2Cx->I2C_DR;
				pHandle->pRxBuffer++;
				pHandle->RxLen--;
			}if(pHandle->RxLen==0)
			{if(pHandle->RepStart==I2C_REP_START_DISABLE)
				{I2C_GenerateStopCondition(pHandle->I2Cx);}
				I2C_CloseReceiveData(pHandle);
				I2C_AppEventCallback(pHandle,I2C_EV_RX_CMPLT);
			}
		}
	}
}

void I2C_CloseSendData(I2C_Handle_t* pHandle)
{
	pHandle->I2Cx->I2C_CR2&=~(1<<I2C_CR2_ITERREN);
	pHandle->I2Cx->I2C_CR2&=~(1<<I2C_CR2_ITBUFEN);
	pHandle->I2Cx->I2C_CR2&=~(1<<I2C_CR2_ITEVTEN);
	pHandle->TxRxState=I2C_READY;
	pHandle->pTxBuffer=NULL;
	pHandle->TxLen=0;
	if(pHandle->Config.I2C_ACKControl==I2C_ACK_ENABLE)
	{
		I2C_HandleACK(pHandle->I2Cx,I2C_ACK_ENABLE);
	}
}
void I2C_CloseReceiveData(I2C_Handle_t* pHandle)
{
	pHandle->I2Cx->I2C_CR2&=~(1<<I2C_CR2_ITERREN);
	pHandle->I2Cx->I2C_CR2&=~(1<<I2C_CR2_ITBUFEN);
	pHandle->I2Cx->I2C_CR2&=~(1<<I2C_CR2_ITEVTEN);
	pHandle->TxRxState=I2C_READY;
	pHandle->pRxBuffer=NULL;
	pHandle->RxLen=0;
	pHandle->RxSize=0;
	if(pHandle->Config.I2C_ACKControl==I2C_ACK_ENABLE)
	{
		I2C_HandleACK(pHandle->I2Cx,I2C_ACK_ENABLE);
	}
}


void I2C_PeriClockControl(I2C_Regdef_t *pI2Cx, uint8_t ENORDI)
{
	if(ENORDI==ENABLE)
	{
		if(pI2Cx==I2C1)
		{
			I2C1_PCLK_EN;
		}else if (pI2Cx==I2C2)
		{
			I2C2_PCLK_EN;
		}else if(pI2Cx==I2C3)
		{
			I2C3_PCLK_EN;
		}
	}
	else
	{
		if(pI2Cx==I2C1)
		{
			I2C1_PCLK_DIS;
		}else if (pI2Cx==I2C2)
		{
			I2C2_PCLK_DIS;
		}else if(pI2Cx==I2C3)
		{
			I2C3_PCLK_DIS;
		}
	}
}

void I2C_Init(I2C_Handle_t* pHandle)
{
	I2C_PeriClockControl(pHandle->I2Cx,ENABLE);
	pHandle->I2Cx->I2C_CR1|=(1<<I2C_CR1_PE);
	uint32_t temp=0;
	temp=pHandle->Config.I2C_ACKControl;
	pHandle->I2Cx->I2C_CR1&=~(temp<<10);		//Clear the ACK field
	pHandle->I2Cx->I2C_CR1|=(temp<<10);			//Set the ackfield
	temp=0;

	temp=APB1_CLK/1000000;
	pHandle->I2Cx->I2C_CR2&=~(0x3F);		//Clear the bitfields
	pHandle->I2Cx->I2C_CR2|=temp;		//Insert the FREQ value
	temp=0;

	temp=(pHandle->Config.I2C_DeviceAddress<<1);
	pHandle->I2Cx->I2C_OAR1|=(1<<14);	//Requested as per data sheet states
	pHandle->I2Cx->I2C_OAR1&=~temp;		//Clear the address for slave mode
	pHandle->I2Cx->I2C_OAR1|=temp;		//Set own address for slave mode
	temp=0;
	if(pHandle->Config.I2C_SCL_SPEED<=I2C_SCL_SPEED_SM)		//Standard mode
	{
		pHandle->I2Cx->I2C_CCR&=~(1<<15);		//Clearing the SM/FM bit for standard mode
		temp=(APB1_CLK)/(2*pHandle->Config.I2C_SCL_SPEED);
		pHandle->I2Cx->I2C_CCR&=~(0x00000FFF<<15);	//Clearing the CCR field
	}else
	{
		pHandle->I2Cx->I2C_CCR|=(1<<15);	//Setting the SM/FM bit for fast mode
		if(pHandle->Config.I2C_FMDutyCycle==I2C_FM_DUTY_2)
		{
			pHandle->I2Cx->I2C_CCR&=~(1<<14);		//Setting dutycycle tlow/thigh=2
			temp=(APB1_CLK)/(3*pHandle->Config.I2C_SCL_SPEED);
			pHandle->I2Cx->I2C_CCR&=~(0x00000FFF);	//Clearing the CCR field
		}else		//I2C_FM_DUTY_16_9
		{
			pHandle->I2Cx->I2C_CCR|=(1<<14);		//Setting dutycycle tlow/thigh=16/9
			temp=(APB1_CLK)/(25*pHandle->Config.I2C_SCL_SPEED);
			pHandle->I2Cx->I2C_CCR&=~(0x00000FFF);	//Clearing the CCR field
		}
	}
	pHandle->I2Cx->I2C_CCR|=temp;			//Setting the clock speed according to S/F mode
	temp=0;

	//Setting TRISE Register
	if(pHandle->Config.I2C_SCL_SPEED<=I2C_SCL_SPEED_SM)
	{
		//standard mode
		temp=APB1_CLK;
		temp=(APB1_CLK/10000000U)+1;


	}else
	{	// fast mode
		temp=APB1_CLK;
		temp=(APB1_CLK*300U)/1000000000+1;
	}
	pHandle->I2Cx->I2C_TRISE=temp&(0x3F);
	temp=0;
}

void I2C_DeInit(I2C_Regdef_t *pI2Cx)
{
	uint32_t *pReset=(uint32_t *)RCC_APB1_RESET;
	if(pI2Cx==I2C1)
	{
		*pReset|=(1<<21);
		*pReset&=~(1<<21);
		I2C1_PCLK_DIS;
	}else if(pI2Cx==I2C2)
	{
		*pReset|=(1<<22);
		*pReset&=~(1<<22);
		I2C2_PCLK_DIS;
	}else if(pI2Cx==I2C3)
	{
		*pReset|=(1<<23);
		*pReset&=~(1<<23);
		I2C3_PCLK_DIS;
	}
}

void I2C_master_senddata(I2C_Regdef_t* pI2Cx, uint8_t *pTxBuffer,uint32_t len, uint8_t SlaveAddr,uint8_t RepStart)
{
	//1. Start condition
	I2C_GenerateStartCondition(pI2Cx);

	//2. Confirming that Start Condition is generated
	while(!I2C_GetFlagStatus(pI2Cx,I2C_SR1_FLAG_SB));

	//3. Sending Address to write to
	I2C_AddressPhase_Write(pI2Cx,SlaveAddr);

	//4. Confirming address phase was complete
	while(!I2C_GetFlagStatus(pI2Cx,I2C_SR1_FLAG_ADDR));

	//5. Clearing address flags to proceed
	I2C_ClearAddrFlag(pI2Cx);

	//6. Send data until len=0
	while (len>0)
	{
		while(!I2C_GetFlagStatus(pI2Cx,I2C_SR1_FLAG_TxE));
		pI2Cx->I2C_DR=*pTxBuffer;
		pTxBuffer++;
		len--;
	}

	//7. Check flags for stop condition availability
	while(!I2C_GetFlagStatus(pI2Cx,I2C_SR1_FLAG_TxE));

	while(!I2C_GetFlagStatus(pI2Cx,I2C_SR1_FLAG_BTF));

	//8. Generate the stop condition if repeated start is disabled
	if(RepStart==I2C_REP_START_DISABLE)
	{I2C_GenerateStopCondition(pI2Cx);}

}

void I2C_master_receivedata(I2C_Regdef_t* pI2Cx, uint8_t *pRxBuffer,uint32_t len, uint8_t SlaveAddr,uint8_t RepStart)
{
	//1. Start condition
	I2C_GenerateStartCondition(pI2Cx);

	//2. Confirming that Start Condition is generated
	while(!I2C_GetFlagStatus(pI2Cx,I2C_SR1_FLAG_SB));

	//3. Sending Address to write to
	I2C_AddressPhase_Read(pI2Cx,SlaveAddr);

	//4. Confirming address phase was complete
	while(!I2C_GetFlagStatus(pI2Cx,I2C_SR1_FLAG_ADDR));

	if(len==1)
	{
		//Disable ACKing
		I2C_HandleACK(pI2Cx,DISABLE);
		//Clear ADDR flag
		I2C_ClearAddrFlag(pI2Cx);

		//Confirm there is data in RX register
		while(!I2C_GetFlagStatus(pI2Cx,I2C_SR1_FLAG_RxNE));

		//Generate stop condition if repeated start is disabled
		if(RepStart==I2C_REP_START_DISABLE)
		{I2C_GenerateStopCondition(pI2Cx);}
		*pRxBuffer=pI2Cx->I2C_DR;
	}
	if (len>1)
	{
		//Clear ADDR flag
		I2C_ClearAddrFlag(pI2Cx);
		for(int i=len;i>0;i--)
		{
			//Confirm there is data in RX register
			while(!I2C_GetFlagStatus(pI2Cx,I2C_SR1_FLAG_RxNE));
			if(i==2)
			{
				//Disable ACKing
				I2C_HandleACK(pI2Cx,DISABLE);
				//Generate stop condition if repeated start is disabled
				if(RepStart==I2C_REP_START_DISABLE)
				{I2C_GenerateStopCondition(pI2Cx);}
			}
			*pRxBuffer=pI2Cx->I2C_DR;
			pRxBuffer++;
		}
	}

	I2C_HandleACK(pI2Cx,ENABLE);	//Re-ENABLE ACK for future transmissions
}

void I2C1_GPIOInit(void)	//Sets PB7 and PB6 as SDA and SCL for I2C1 relatively
{
	GPIO_Handle_t pHandle;
	pHandle.GPIOx=GPIOB;
	//Configuring PB6 as SCL
	pHandle.Config.GPIO_PinNumber=6;
	pHandle.Config.GPIO_Mode=GPIO_MODE_ALT_FUNC;
	pHandle.Config.GPIO_OutputType=GPIO_OTYPE_OD;
	pHandle.Config.GPIO_OutputSpeed=GPIO_OSPEED_HIGH;
	pHandle.Config.GPIO_PUPD=GPIO_PUPD_PU;
	pHandle.Config.GPIO_AlternateFunc=GPIO_ALT_FUNC4;
	GPIO_PeriClockControl(pHandle.GPIOx, ENABLE);
	GPIO_Init(&pHandle);
	//Configuring PB7 as SDA
	pHandle.Config.GPIO_PinNumber=7;
	GPIO_Init(&pHandle);
}

void I2C1_GPIODeInit(void)	//Sets PB7 and PB8 as SDA and SCA for I2C1 relatively
{
	GPIO_Handle_t pHandle;
	pHandle.GPIOx=GPIOB;
	//Configuring PB6 as default Input pin
	pHandle.Config.GPIO_PinNumber=6;
	pHandle.Config.GPIO_Mode=GPIO_MODE_INPUT;
	pHandle.Config.GPIO_OutputType=GPIO_OTYPE_RESET;
	pHandle.Config.GPIO_OutputSpeed=GPIO_OSPEED_RESET;
	pHandle.Config.GPIO_PUPD=GPIO_PUPD_RESET;
	pHandle.Config.GPIO_AlternateFunc=GPIO_ALT_FUNC0;
	GPIO_Init(&pHandle);
	//Configuring PB7 as default Input pin
	pHandle.Config.GPIO_PinNumber=7;
	GPIO_Init(&pHandle);
}


void I2C1_InterruptEnable(void)
{
	uint32_t *pISER=(uint32_t *)NVIC_ISER_BASEADDR;
	*pISER|=(1<<31);
	pISER++;
	*pISER|=(1<<0);
}
void I2C1_InterruptDisable(void)
{
	uint32_t *pICER=(uint32_t *)NVIC_ICER_BASEADDR;
	*pICER&=~(1<<31);
	pICER++;
	*pICER&=~(1<<0);
}

void I2C1_InterruptPriority(uint8_t Priority)
{
	uint32_t *pIPR=(uint32_t *)NVIC_IPR_BASEADDR;
	uint8_t temp=31/4;
	pIPR+=temp;
	temp%=4;
	*pIPR&=~(0xF<<(temp*8+4)); //Clearing the priority for IRQ31
	*pIPR|=((Priority&0xF)<<(temp*8+4)); //Setting the priority for IRQ31

	pIPR++;
	temp=0;
	*pIPR&=~(0xF<<4); //Clearing the priority for IRQ32
	*pIPR|=((Priority&0xF)<<4); //Setting the priority for IRQ32
}

uint8_t I2C_master_senddata_IT(I2C_Handle_t* pHandle, uint8_t *pTxBuffer,uint32_t len, uint8_t SlaveAddr,uint8_t RepStart)
{
	uint8_t busy=pHandle->TxRxState;
	if((busy!=I2C_BUSY_RX)&&(busy!=I2C_BUSY_TX))
	{
		pHandle->pTxBuffer=pTxBuffer;
		pHandle->TxLen=len;
		pHandle->TxRxState=I2C_BUSY_TX;
		pHandle->SlaveAddr=SlaveAddr;
		pHandle->RepStart=RepStart;


		pHandle->I2Cx->I2C_CR2|=(1<<I2C_CR2_ITERREN);
		pHandle->I2Cx->I2C_CR2|=(1<<I2C_CR2_ITBUFEN);
		I2C_GenerateStartCondition(pHandle->I2Cx);
		pHandle->I2Cx->I2C_CR2|=(1<<I2C_CR2_ITEVTEN);
	}
	return busy;
}


uint8_t I2C_master_receivedata_IT(I2C_Handle_t* pHandle, uint8_t *pRxBuffer,uint32_t len, uint8_t SlaveAddr,uint8_t RepStart)
{
		uint8_t busy=pHandle->TxRxState;
		if((busy!=I2C_BUSY_RX)&&(busy!=I2C_BUSY_TX))
		{
			pHandle->pRxBuffer=pRxBuffer;
			pHandle->RxLen=len;
			pHandle->RxSize=len;
			pHandle->TxRxState=I2C_BUSY_RX;
			pHandle->SlaveAddr=SlaveAddr;
			pHandle->RepStart=RepStart;


			pHandle->I2Cx->I2C_CR2|=(1<<I2C_CR2_ITERREN);
			pHandle->I2Cx->I2C_CR2|=(1<<I2C_CR2_ITBUFEN);
			I2C_GenerateStartCondition(pHandle->I2Cx);
			pHandle->I2Cx->I2C_CR2|=(1<<I2C_CR2_ITEVTEN);
		}
		return busy;
}

void I2C_EV_IRQHandling (I2C_Handle_t* pHandle)
{
	uint32_t temp1,temp2,temp3;
	temp1=pHandle->I2Cx->I2C_CR2&(1<<I2C_CR2_ITEVTEN);
	temp2=pHandle->I2Cx->I2C_CR2&(1<<I2C_CR2_ITBUFEN);
	temp3=pHandle->I2Cx->I2C_SR1&I2C_SR1_FLAG_SB;
	if(temp1&&temp3)
	{
		//handle SB condition
		if(pHandle->TxRxState==I2C_BUSY_TX)
		{
			I2C_AddressPhase_Write(pHandle->I2Cx,pHandle->SlaveAddr);
		}else if(pHandle->TxRxState==I2C_BUSY_RX)
		{
			I2C_AddressPhase_Read(pHandle->I2Cx,pHandle->SlaveAddr);
		}
	}
	temp3=pHandle->I2Cx->I2C_SR1&(I2C_SR1_FLAG_ADDR);
	if(temp1&&temp3)
	{
		//handle Address condition
		if(pHandle->TxRxState==I2C_BUSY_RX)
		{
			if(pHandle->RxSize==1)
			{
				I2C_HandleACK(pHandle->I2Cx,I2C_ACK_DISABLE);
			}
		}

		I2C_ClearAddrFlag(pHandle->I2Cx);
	}
	temp3=pHandle->I2Cx->I2C_SR1&(I2C_SR1_FLAG_BTF);
	if(temp1&&temp3)
	{
		//handle BTF condition
		if(pHandle->TxRxState==I2C_BUSY_TX)
		{
			if(I2C_GetFlagStatus(pHandle->I2Cx,I2C_SR1_FLAG_TxE))
			{
				//both TXE=1 and BTF=1
				if(pHandle->TxLen==0)
				{
				I2C_GenerateStopCondition(pHandle->I2Cx);
				I2C_CloseSendData(pHandle);
				I2C_AppEventCallback(pHandle,I2C_EV_TX_CMPLT);
				}
			}

		}

	}
	temp3=pHandle->I2Cx->I2C_SR1&(I2C_SR1_FLAG_TxE);
	if(temp1&&temp2&&temp3)
	{
		//handle TxE condition
		if(I2C_GetFlagStatus(pHandle->I2Cx,I2C_SR2_FLAG_MSL))	//Check for master mode
		{
			if(pHandle->TxRxState==I2C_BUSY_TX)
			{
				I2C_TxEIRQHandler(pHandle);
			}
		}

	}
	temp3=pHandle->I2Cx->I2C_SR1&(I2C_SR1_FLAG_RxNE);
	if(temp1&&temp2&&temp3)
	{
		//handle RxNE condition
		if(I2C_GetFlagStatus(pHandle->I2Cx,I2C_SR2_FLAG_MSL))	//Check for master mode
		{
			if(pHandle->TxRxState==I2C_BUSY_RX)
			{
				I2C_RxNEIRQHandler(pHandle);
			}
		}
	}
}
void I2C_ER_IRQHandling(I2C_Handle_t *pHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pHandle->I2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pHandle->I2Cx->I2C_SR1) & (I2C_SR1_FLAG_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the bus error flag
		pHandle->I2Cx->I2C_SR1 &= ~(I2C_SR1_FLAG_BERR);

		//Implement the code to notify the application about the error
	   I2C_AppEventCallback(pHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pHandle->I2Cx->I2C_SR1) & (I2C_SR1_FLAG_ARLO);
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pHandle->I2Cx->I2C_SR1 &= ~(I2C_SR1_FLAG_ARLO);
		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pHandle->I2Cx->I2C_SR1) & (I2C_SR1_FLAG_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pHandle->I2Cx->I2C_SR1 &= ~(I2C_SR1_FLAG_AF);
		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pHandle->I2Cx->I2C_SR1) & (I2C_SR1_FLAG_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pHandle->I2Cx->I2C_SR1 &= ~(I2C_SR1_FLAG_OVR);
		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pHandle->I2Cx->I2C_SR1) & (I2C_SR1_FLAG_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pHandle->I2Cx->I2C_SR1 &= ~(I2C_SR1_FLAG_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pHandle,I2C_ERROR_TIMEOUT);
	}
}


#endif /* I2C_DRIVER_C_ */
