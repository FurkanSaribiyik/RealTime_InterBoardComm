/*
 * I2C_Driver.h
 *
 *  Created on: Jan 31, 2024
 *      Author: Furkan
 */

#ifndef INC_I2C_DRIVER_H_
#define INC_I2C_DRIVER_H_

#include "GPIO_driver.h"
#include <string.h>

#define HSI_CLK 16000000U
#define APB1_CLK HSI_CLK

#define I2C1_BASEADDR 0x40005400U
#define I2C2_BASEADDR 0x40005800U
#define I2C3_BASEADDR 0x40005C00U

#define RCC_APB1_ENR (RCC_BASEADDR+0X40)
#define RCC_APB1_RESET (RCC_BASEADDR+0X20)


#define I2C1_PCLK_EN (*(uint32_t*)(RCC_APB1_ENR)|=(1<<21))
#define I2C2_PCLK_EN (*(uint32_t*)(RCC_APB1_ENR)|=(1<<22))
#define I2C3_PCLK_EN (*(uint32_t*)(RCC_APB1_ENR)|=(1<<23))
#define I2C1_PCLK_DIS (*(uint32_t*)(RCC_APB1_ENR)&=~(1<<21))
#define I2C2_PCLK_DIS (*(uint32_t*)(RCC_APB1_ENR)&=~(1<<22))
#define I2C3_PCLK_DIS (*(uint32_t*)(RCC_APB1_ENR)&=~(1<<23))


typedef struct{
	uint32_t I2C_CR1;
	uint32_t I2C_CR2;
	uint32_t I2C_OAR1;
	uint32_t I2C_OAR2;
	uint32_t I2C_DR;
	uint32_t I2C_SR1;
	uint32_t I2C_SR2;
	uint32_t I2C_CCR;
	uint32_t I2C_TRISE;
	uint32_t I2C_FLTR;
}I2C_Regdef_t;

typedef struct{
	uint32_t I2C_SCL_SPEED;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_Regdef_t* I2Cx;
	I2C_Config_t Config;
	//interrupt related operators
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t SlaveAddr;
	uint32_t RxSize;
	uint8_t RepStart;
}I2C_Handle_t;

#define I2C_SCL_SPEED_SM 100000U
#define I2C_SCL_SPEED_FM2K 200000U
#define I2C_SCL_SPEED_FM4K 400000U

#define I2C_ACK_DISABLE 0
#define I2C_ACK_ENABLE 1

#define I2C_FLAG_RESET 0
#define I2C_FLAG_SET 1

#define I2C_MASTER_WRITE 0
#define I2C_MASTER_READ 1


#define I2C_FM_DUTY_2 0
#define I2C_FM_DUTY_16_9 1

#define I2C1 (I2C_Regdef_t*)(I2C1_BASEADDR)
#define I2C2 (I2C_Regdef_t*)(I2C2_BASEADDR)
#define I2C3 (I2C_Regdef_t*)(I2C3_BASEADDR)

#define I2C_CR1_PE 0
#define I2C_CR1_START 8
#define I2C_CR1_STOP 9
#define I2C_CR1_ACK 10

#define I2C_CR2_ITERREN 8
#define I2C_CR2_ITEVTEN 9
#define I2C_CR2_ITBUFEN 10

#define I2C_SR1_FLAG_SB (1<<0)
#define I2C_SR1_FLAG_ADDR (1<<1)
#define I2C_SR1_FLAG_BTF (1<<2)
#define I2C_SR1_FLAG_ADD10 (1<<3)
#define I2C_SR1_FLAG_STOPF (1<<4)
#define I2C_SR1_FLAG_RxNE (1<<6)
#define I2C_SR1_FLAG_TxE (1<<7)
#define I2C_SR1_FLAG_BERR (1<<8)
#define I2C_SR1_FLAG_ARLO (1<<9)
#define I2C_SR1_FLAG_AF (1<<10)
#define I2C_SR1_FLAG_OVR (1<<11)
#define I2C_SR1_FLAG_PECERR (1<<12)
#define I2C_SR1_FLAG_TIMEOUT (1<<14)
#define I2C_SR1_FLAG_SMBALERT (1<<15)

#define I2C_SR2_FLAG_MSL (1<<16)
#define I2C_SR2_FLAG_BUSY (1<<17)
#define I2C_SR2_FLAG_TRA (1<<18)
#define I2C_SR2_FLAG_GENCALL (1<<20)
#define I2C_SR2_FLAG_SMBDEFAULT (1<<21)
#define I2C_SR2_FLAG_SMBHOST (1<<22)
#define I2C_SR2_FLAG_DUALF (1<<23)
#define I2C_SR2_FLAG_PEC (FF<<24)

#define I2C_REP_START_DISABLE DISABLE
#define I2C_REP_START_ENABLE ENABLE

#define I2C_READY 0
#define I2C_BUSY_TX 1
#define I2C_BUSY_RX 2

#define I2C_EV_TX_CMPLT 0
#define I2C_EV_RX_CMPLT 1
#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7

void I2C_PeriClockControl(I2C_Regdef_t *pI2Cx, uint8_t ENORDI);
void I2C_Init(I2C_Handle_t* pHandle);
void I2C_DeInit(I2C_Regdef_t *pI2Cx);

void I2C_master_senddata(I2C_Regdef_t* pI2Cx, uint8_t *pTxBuffer,uint32_t len, uint8_t SlaveAddr,uint8_t RepStart);
void I2C_master_receivedata(I2C_Regdef_t* pI2Cx, uint8_t *pRxBuffer,uint32_t len, uint8_t SlaveAddr,uint8_t RepStart);

void I2C1_InterruptEnable(void);
void I2C1_InterruptDisable(void);

void I2C1_InterruptPriority(uint8_t Priority);

uint8_t I2C_master_senddata_IT(I2C_Handle_t* pHandle, uint8_t *pTxBuffer,uint32_t len, uint8_t SlaveAddr,uint8_t RepStart);
uint8_t I2C_master_receivedata_IT(I2C_Handle_t* pHandle, uint8_t *pRxBuffer,uint32_t len, uint8_t SlaveAddr,uint8_t RepStart);

void I2C_EV_IRQHandling (I2C_Handle_t* pHandle);
void I2C_ER_IRQHandling (I2C_Handle_t* pHandle);


void I2C_AppEventCallback(I2C_Handle_t* pHandle,uint8_t Event);

void I2C_CloseSendData(I2C_Handle_t* pHandle);
void I2C_CloseReceiveData(I2C_Handle_t* pHandle);
void I2C_GenerateStopCondition(I2C_Regdef_t* pI2Cx);

void I2C1_GPIOInit(void);
void I2C1_GPIODeInit(void);

#endif /* INC_I2C_DRIVER_H_ */
