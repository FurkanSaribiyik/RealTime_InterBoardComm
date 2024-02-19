/*
 * GPIO_driver.h
 *
 *  Created on: Jan 30, 2024
 *      Author: Furkan
 */

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

#include <stdint.h>

#define HSI_CLK 16000000U

#define DISABLE 0
#define ENABLE 1

#define RESET 0
#define SET 1

#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1

#define NVIC_ISER_BASEADDR 0xE000E100
#define NVIC_ICER_BASEADDR 0XE000E180
#define NVIC_IPR_BASEADDR 0xE000E400

#define AHB1_BASEADDR 0x40020000U

#define GPIOA_BASEADDR 0x40020000U
#define GPIOB_BASEADDR 0x40020400U
#define GPIOC_BASEADDR 0x40020800U
#define GPIOD_BASEADDR 0x40020C00U
#define GPIOE_BASEADDR 0x40021000U
#define GPIOF_BASEADDR 0x40021400U
#define GPIOG_BASEADDR 0x40021800U
#define GPIOH_BASEADDR 0x40021C00U
#define GPIOI_BASEADDR 0x40022000U

#define RCC_BASEADDR 0x40023800U
#define RCC_AHB1RESET (RCC_BASEADDR+0x10)
#define RCC_AHB1ENR (RCC_BASEADDR+0x30)

#define GPIOA_PCLK_EN (*(uint32_t*)(RCC_AHB1ENR)|=(1<<0))
#define GPIOB_PCLK_EN (*(uint32_t*)(RCC_AHB1ENR)|=(1<<1))
#define GPIOC_PCLK_EN (*(uint32_t*)(RCC_AHB1ENR)|=(1<<2))
#define GPIOD_PCLK_EN (*(uint32_t*)(RCC_AHB1ENR)|=(1<<3))
#define GPIOE_PCLK_EN (*(uint32_t*)(RCC_AHB1ENR)|=(1<<4))
#define GPIOF_PCLK_EN (*(uint32_t*)(RCC_AHB1ENR)|=(1<<5))
#define GPIOG_PCLK_EN (*(uint32_t*)(RCC_AHB1ENR)|=(1<<6))
#define GPIOH_PCLK_EN (*(uint32_t*)(RCC_AHB1ENR)|=(1<<7))
#define GPIOI_PCLK_EN (*(uint32_t*)(RCC_AHB1ENR)|=(1<<8))

#define GPIOA_PCLK_DIS (*(uint32_t*)(RCC_AHB1ENR)&=~(1<<0))
#define GPIOB_PCLK_DIS (*(uint32_t*)(RCC_AHB1ENR)&=~(1<<1))
#define GPIOC_PCLK_DIS (*(uint32_t*)(RCC_AHB1ENR)&=~(1<<2))
#define GPIOD_PCLK_DIS (*(uint32_t*)(RCC_AHB1ENR)&=~(1<<3))
#define GPIOE_PCLK_DIS (*(uint32_t*)(RCC_AHB1ENR)&=~(1<<4))
#define GPIOF_PCLK_DIS (*(uint32_t*)(RCC_AHB1ENR)&=~(1<<5))
#define GPIOG_PCLK_DIS (*(uint32_t*)(RCC_AHB1ENR)&=~(1<<6))
#define GPIOH_PCLK_DIS (*(uint32_t*)(RCC_AHB1ENR)&=~(1<<7))
#define GPIOI_PCLK_DIS (*(uint32_t*)(RCC_AHB1ENR)&=~(1<<8))



#define GPIO_MODE_RESET 0
#define GPIO_MODE_INPUT	GPIO_MODE_RESET
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALT_FUNC 2
#define GPIO_MODE_ANALOG 3

#define GPIO_OTYPE_RESET 0
#define GPIO_OTYPE_PP GPIO_OTYPE_RESET
#define GPIO_OTYPE_OD 1

#define GPIO_OSPEED_RESET 0
#define GPIO_OSPEED_LOW GPIO_OSPEED_RESET
#define GPIO_OSPEED_MED 1
#define GPIO_OSPEED_HIGH 2
#define GPIO_OSPEED_VERY_HIGH 3

#define GPIO_PUPD_RESET 0
#define GPIO_PUPD_NO_PUPD GPIO_PUPD_RESET
#define GPIO_PUPD_PU 1
#define GPIO_PUPD_PD 2

#define GPIO_ALT_FUNC0 0
#define GPIO_ALT_FUNC1 1
#define GPIO_ALT_FUNC2 2
#define GPIO_ALT_FUNC3 3
#define GPIO_ALT_FUNC4 4
#define GPIO_ALT_FUNC5 5
#define GPIO_ALT_FUNC6 6
#define GPIO_ALT_FUNC7 7
#define GPIO_ALT_FUNC8 8
#define GPIO_ALT_FUNC9 9
#define GPIO_ALT_FUNC10 10
#define GPIO_ALT_FUNC11 11
#define GPIO_ALT_FUNC12 12
#define GPIO_ALT_FUNC13 13
#define GPIO_ALT_FUNC14 14
#define GPIO_ALT_FUNC15 15

typedef struct{
	uint32_t GPIO_MODER;
	uint32_t GPIO_OTYPER;
	uint32_t GPIO_OSPEEDR;
	uint32_t GPIO_PUPDR;
	uint32_t GPIO_IDR;
	uint32_t GPIO_ODR;
	uint32_t GPIO_BSRR;;
	uint32_t GPIO_LCKR;
	uint32_t GPIO_AFRL;
	uint32_t GPIO_AFRH;
}GPIO_Regdef_t;

#define GPIOA	((GPIO_Regdef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_Regdef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_Regdef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_Regdef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_Regdef_t*)GPIOE_BASEADDR)
#define GPIOF	((GPIO_Regdef_t*)GPIOF_BASEADDR)
#define GPIOG	((GPIO_Regdef_t*)GPIOG_BASEADDR)
#define GPIOH	((GPIO_Regdef_t*)GPIOH_BASEADDR)
#define GPIOI	((GPIO_Regdef_t*)GPIOI_BASEADDR)


typedef struct{
	 uint8_t GPIO_PinNumber;
	 uint8_t GPIO_Mode;
	 uint8_t GPIO_OutputType;
	 uint8_t GPIO_OutputSpeed;
	 uint8_t GPIO_PUPD;
	 uint8_t GPIO_AlternateFunc;
}GPIO_Config_t;

typedef struct{
	GPIO_Regdef_t* GPIOx;
	GPIO_Config_t Config;
}GPIO_Handle_t;

void GPIO_PeriClockControl(GPIO_Regdef_t *pGPIOx, uint8_t ENORDI);
void GPIO_Init(GPIO_Handle_t *Handle);
void GPIO_DeInit(GPIO_Regdef_t *pGPIOx);
uint8_t ReadFromInputPin(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber);
uint16_t ReadFromInputPort(GPIO_Regdef_t *pGPIOx, uint8_t PinNumber);
void WriteToOutputPin(GPIO_Regdef_t *pGPIOx,uint8_t PinNumber, uint8_t val);
void WriteToOutputPort(GPIO_Regdef_t *pGPIOx,uint16_t val);

#include "i2c_driver.h"
#endif /* GPIO_DRIVER_H_ */
