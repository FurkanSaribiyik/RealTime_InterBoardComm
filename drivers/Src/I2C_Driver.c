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
static void I2C_GenerateStopCondition(I2C_Regdef_t* pI2Cx);
static void I2C_HandleACK(I2C_Regdef_t* pI2Cx, uint8_t ENORDI);

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

static void I2C_GenerateStopCondition(I2C_Regdef_t* pI2Cx)
{
	pI2Cx->I2C_CR1|=(1<<I2C_CR1_STOP);
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

void I2C1_GPIOInit(void)	//Sets PB7 and PB8 as SDA and SCA for I2C1 relatively
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


#endif /* I2C_DRIVER_C_ */
