/*
 * GPIO_driver.h
 *
 *  Created on: Jan 30, 2024
 *      Author: Furkan
 */

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

#include <stdint.h>

#define DISABLE 0
#define ENABLE 1

#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1

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
	volatile uint32_t GPIO_MODER;
	volatile uint32_t GPIO_OTYPER;
	volatile uint32_t GPIO_OSPEEDR;
	volatile uint32_t GPIO_PUPDR;
	volatile uint32_t GPIO_IDR;
	volatile uint32_t GPIO_ODR;
	volatile uint32_t GPIO_BSRR;;
	volatile uint32_t GPIO_LCKR;
	volatile uint32_t GPIO_AFRL;
	volatile uint32_t GPIO_AFRH;
}GPIO_Regs;

#define GPIOA	((GPIO_Regs*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_Regs*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_Regs*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_Regs*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_Regs*)GPIOE_BASEADDR)
#define GPIOF	((GPIO_Regs*)GPIOF_BASEADDR)
#define GPIOG	((GPIO_Regs*)GPIOG_BASEADDR)
#define GPIOH	((GPIO_Regs*)GPIOH_BASEADDR)
#define GPIOI	((GPIO_Regs*)GPIOI_BASEADDR)


typedef struct{
	volatile uint8_t GPIO_PinNumber;
	volatile uint8_t GPIO_Mode;
	volatile uint8_t GPIO_OutputType;
	volatile uint8_t GPIO_OutputSpeed;
	volatile uint8_t GPIO_PUPD;
	volatile uint8_t GPIO_AlternateFunc;
}GPIO_Config;

typedef struct{
	volatile GPIO_Regs* GPIOx;
	volatile GPIO_Config Config;
}GPIO_Handle;

void GPIO_PeriClockControl(GPIO_Regs *pGPIOx, uint8_t ENORDI);
void GPIO_Init(GPIO_Handle *Handle);
void GPIO_DeInit(GPIO_Regs *pGPIOx);
uint8_t ReadFromInputPin(GPIO_Regs *pGPIOx, uint8_t PinNumber);
uint16_t ReadFromInputPort(GPIO_Regs *pGPIOx, uint8_t PinNumber);
void WriteToOutputPin(GPIO_Regs *pGPIOx,uint8_t PinNumber, uint8_t val);
void WriteToOutputPort(GPIO_Regs *pGPIOx,uint16_t val);


#endif /* GPIO_DRIVER_H_ */
