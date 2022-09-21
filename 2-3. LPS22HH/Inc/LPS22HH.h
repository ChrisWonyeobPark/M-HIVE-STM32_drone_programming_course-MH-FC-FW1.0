/**
 * LPS22HH.c
 * @author ChrisP @ M-HIVE

 * This library source code has been created for STM32F4. Only supports SPI.
 *
 * Development environment specifics:
 * STM32CubeIDE 1.0.0
 * STM32CubeF4 FW V1.24.1
 * STM32F4 LL Driver(SPI)
 *
 * Created by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, July, 2019
 * Rev. 1.0
 *
 * https://github.com/ChrisWonyeobPark
 * https://blog.naver.com/lbiith
 * https://cafe.naver.com/mhiveacademy
*/

#ifndef	_LPS22HH_H
#define	_LPS22HH_H

#include "main.h"
//////////////////////////////////////////////////////////////////////////

/*
SPI Operational Features
1. Data is delivered MSB first and LSB last
2. Data is latched on the rising edge of SPC
3. Data should be transitioned on the falling edge of SPC
4. The maximum frequency of SPC is 10MHz
5. SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). The first byte contains the
SPI Address, and the following byte(s) contain(s) the SPI data. The first bit of the first byte contains the Read/Write bit
and indicates the Read (1) or Write (0) operation. The following 7 bits contain the Register Address. In cases of multiplebyte Read/Writes, data is two or more bytes:
*/

/**
 * @brief Definition for connected to SPI3 (APB1 = 42MHz)
 */
#define LPS22HH_SPI_CHANNEL			SPI3

#define LPS22HH_SPI_SCLK_PIN		LL_GPIO_PIN_3
#define LPS22HH_SPI_SCLK_PORT		GPIOB
#define LPS22HH_SPI_SCLK_CLK		LL_AHB1_GRP1_PERIPH_GPIOB

#define LPS22HH_SPI_MISO_PIN		LL_GPIO_PIN_4
#define LPS22HH_SPI_MISO_PORT		GPIOB
#define LPS22HH_SPI_MISO_CLK		LL_AHB1_GRP1_PERIPH_GPIOB

#define LPS22HH_SPI_MOSI_PIN		LL_GPIO_PIN_5
#define LPS22HH_SPI_MOSI_PORT		GPIOB
#define LPS22HH_SPI_MOSI_CLK		LL_AHB1_GRP1_PERIPH_GPIOB

#define LPS22HH_SPI_CS_PIN			LL_GPIO_PIN_6
#define LPS22HH_SPI_CS_PORT			GPIOB
#define LPS22HH_SPI_CS_CLK			LL_AHB1_GRP1_PERIPH_GPIOB

#define LPS22HH_INT_PIN				LL_GPIO_PIN_7
#define LPS22HH_INT_PORT			GPIOB
#define LPS22HH_INT_CLK				LL_AHB1_GRP1_PERIPH_GPIOB
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define CHIP_SELECT(LPS22HH)		LL_GPIO_ResetOutputPin(LPS22HH_SPI_CS_PORT, LPS22HH_SPI_CS_PIN)
#define CHIP_DESELECT(LPS22HH)		LL_GPIO_SetOutputPin(LPS22HH_SPI_CS_PORT, LPS22HH_SPI_CS_PIN)
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


/**
 * @brief LPS22HH Register Map
 */
 
#define	INTERRUPT_CFG	0x0B
#define	THS_P_L			0x0C
#define	THS_P_H			0x0D
#define	IF_CTRL			0x0E
#define	WHO_AM_I		0x0F
#define	CTRL_REG1		0x10
#define	CTRL_REG2		0x11
#define	CTRL_REG3		0x12
#define	FIFO_CTRL		0x13
#define	FIFO_WTM		0x14
#define	REF_P_L			0x15
#define	REF_P_H			0x16
//	Reserved			0x17
#define	RPDS_L			0x18
#define	RPDS_H			0x19
//	Reserved			0x1A-0x23
#define	INT_SOURCE		0x24
#define	FIFO_STATUS1	0x25
#define	FIFO_STATUS2	0x26
#define	STATUS			0x27
#define	PRESSURE_OUT_XL	0x28
#define	PRESSURE_OUT_L	0x29
#define	PRESSURE_OUT_H	0x2A
#define	TEMP_OUT_L		0x2B
#define	TEMP_OUT_H		0x2C
//	Reserved			0x2D-0x77
#define	FIFO_DATA_OUT_PRESS_XL	0x78
#define	FIFO_DATA_OUT_PRESS_L	0x79
#define	FIFO_DATA_OUT_PRESS_H	0x7A
#define	FIFO_DATA_OUT_TEMP_L	0x7B
#define	FIFO_DATA_OUT_TEMP_H	0x7C



typedef struct _LPS22HH{
	int32_t pressure_raw;
	int16_t temperature_raw;
	float baroAlt;
	float baroAltFilt;
}Struct_LPS22HH;

extern Struct_LPS22HH LPS22HH;

int LPS22HH_Initialization(void);
int LPS22HH_DataReady(void);
void LPS22HH_GetPressure(int32_t* pressure);
void LPS22HH_GetTemperature(int16_t* temperature);
float getAltitude1(float pressure); //No temperature correction.
float getAltitude2(float pressure, float temperature); //Get Altitude with temperature correction.

#endif
