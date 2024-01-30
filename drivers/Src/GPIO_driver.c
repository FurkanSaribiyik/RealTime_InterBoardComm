/*
 * GPIO_driver.c
 *
 *  Created on: Jan 30, 2024
 *      Author: Furkan
 */

#ifndef GPIO_DRIVER_C_
#define GPIO_DRIVER_C_

#include "GPIO_driver.h"

void GPIO_PeriClockControl(GPIO_Regs *pGPIOx, uint8_t ENORDI)
{
	if (ENORDI==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN;
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN;
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN;
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN;
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN;
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_EN;
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_EN;
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN;
		}
		else if(pGPIOx==GPIOI)
		{
			GPIOI_PCLK_EN;
		}
	}
	else
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DIS;
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DIS;
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DIS;
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DIS;
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_DIS;
		}
		else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_DIS;
		}
		else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_DIS;
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_DIS;
		}
		else if(pGPIOx==GPIOI)
		{
			GPIOI_PCLK_DIS;
		}
	}
}

void GPIO_Init(GPIO_Handle *Handle)
{
	uint32_t temp=0;
	//setting the mode of a peripheral
	temp=Handle->Config.GPIO_Mode<<(2*Handle->Config.GPIO_PinNumber);
	Handle->GPIOx->GPIO_MODER&=~(3<<(2*Handle->Config.GPIO_PinNumber));	//Clearing the necessary bitfields
	Handle->GPIOx->GPIO_MODER|=temp;
	temp=0;
	//output type
	temp=Handle->Config.GPIO_OutputType<<Handle->Config.GPIO_PinNumber;
	Handle->GPIOx->GPIO_OTYPER&=~(1<<Handle->Config.GPIO_PinNumber);
	Handle->GPIOx->GPIO_OTYPER|=temp;
	temp=0;
	//output speed
	temp=Handle->Config.GPIO_OutputSpeed<<(2*Handle->Config.GPIO_PinNumber);
	Handle->GPIOx->GPIO_OSPEEDR&=~(3<<(2*Handle->Config.GPIO_PinNumber));	//Clearing the necessary bitfields
	Handle->GPIOx->GPIO_OSPEEDR|=temp;
	temp=0;
	//pull up pull down
	temp=Handle->Config.GPIO_PUPD<<(2*Handle->Config.GPIO_PinNumber);
	Handle->GPIOx->GPIO_PUPDR&=~(3<<(2*Handle->Config.GPIO_PinNumber));	//Clearing the necessary bitfields
	Handle->GPIOx->GPIO_PUPDR|=temp;
	temp=0;
	//alternate func
	if(Handle->Config.GPIO_Mode==GPIO_MODE_ALT_FUNC)
	{
		if(Handle->Config.GPIO_PinNumber<8)
		{
			temp=Handle->Config.GPIO_AlternateFunc<<(4*(Handle->Config.GPIO_PinNumber));
			Handle->GPIOx->GPIO_AFRL&=~(15<<(4*(Handle->Config.GPIO_PinNumber))); //Clearing the necessary bitfields
			Handle->GPIOx->GPIO_AFRL|=temp;
			temp=0;
		}
		else if(Handle->Config.GPIO_PinNumber>=8)
		{
			temp=Handle->Config.GPIO_AlternateFunc<<(4*(Handle->Config.GPIO_PinNumber-8));
			Handle->GPIOx->GPIO_AFRH&=~(15<<(4*(Handle->Config.GPIO_PinNumber-8))); //Clearing the necessary bitfields
			Handle->GPIOx->GPIO_AFRH|=temp;
			temp=0;
		}
	}


}


void GPIO_DeInit(GPIO_Regs *pGPIOx)
{
	uint32_t *pReset=(uint32_t *)RCC_AHB1RESET;
	if(pGPIOx==GPIOA)
	{
		*pReset|=(1<<0);
		*pReset&=~(1<<0);
		GPIOA_PCLK_DIS;
	}else if(pGPIOx==GPIOB)
	{
		*pReset|=(1<<1);
		*pReset&=~(1<<1);
		GPIOB_PCLK_DIS;
	}else if(pGPIOx==GPIOC)
	{
		*pReset|=(1<<2);
		*pReset&=~(1<<2);
		GPIOC_PCLK_DIS;
	}else if(pGPIOx==GPIOD)
	{
		*pReset|=(1<<3);
		*pReset&=~(1<<3);
		GPIOD_PCLK_DIS;
	}else if(pGPIOx==GPIOE)
	{
		*pReset|=(1<<4);
		*pReset&=~(1<<4);
		GPIOE_PCLK_DIS;
	}else if(pGPIOx==GPIOF)
	{
		*pReset|=(1<<5);
		*pReset&=~(1<<5);
		GPIOF_PCLK_DIS;
	}else if(pGPIOx==GPIOG)
	{
		*pReset|=(1<<6);
		*pReset&=~(1<<6);
		GPIOG_PCLK_DIS;
	}else if(pGPIOx==GPIOH)
	{
		*pReset|=(1<<7);
		*pReset&=~(1<<7);
		GPIOH_PCLK_DIS;
	}else if(pGPIOx==GPIOI)
	{
		*pReset|=(1<<8);
		*pReset&=~(1<<8);
		GPIOI_PCLK_DIS;
	}
}


uint8_t ReadFromInputPin(GPIO_Regs *pGPIOx, uint8_t PinNumber)
{
	uint8_t data=(uint8_t)(pGPIOx->GPIO_IDR>>PinNumber)&(0x00000001);
	return data;
}

uint16_t ReadFromInputPort(GPIO_Regs *pGPIOx, uint8_t PinNumber)
{
	uint16_t data=pGPIOx->GPIO_IDR;
	return data;
}

void WriteToOutputPin(GPIO_Regs *pGPIOx,uint8_t PinNumber, uint8_t val)
{
	if(val==GPIO_PIN_SET)
	{
		pGPIOx->GPIO_ODR|=(1<<PinNumber);
	}
	else
	{
		pGPIOx->GPIO_ODR&=~(1<<PinNumber);
	}
}

void WriteToOutputPort(GPIO_Regs *pGPIOx,uint16_t val)
{
	pGPIOx->GPIO_ODR=val;
}


#endif /* GPIO_DRIVER_C_ */
