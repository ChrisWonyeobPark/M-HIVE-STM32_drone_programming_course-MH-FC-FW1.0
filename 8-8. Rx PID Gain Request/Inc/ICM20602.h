/**

 * ICM20602.c
 * @author ChrisP @ M-HIVE

 * This library source code has been created for STM32F4. Only supports SPI.
 *
 * Development environment specifics:
 * STM32CubeIDE 1.0.0
 * STM32CubeF4 FW V1.24.1
 * STM32F4 LL Driver(SPI) and HAL Driver(RCC for HAL_Delay() function)
 *
 * Created by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, July, 2019
 * Rev. 1.0
 *
 * https://github.com/ChrisWonyeobPark/
*/

#ifndef _ICM20602_H
#define _ICM20602_H

#include "main.h"

/*
ICM-20602 SPI Operational Features
1. Data is delivered MSB first and LSB last
2. Data is latched on the rising edge of SPC
3. Data should be transitioned on the falling edge of SPC
4. The maximum frequency of SPC is 10MHz
5. SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). The first byte contains the
SPI Address, and the following byte(s) contain(s) the SPI data. The first bit of the first byte contains the Read/Write bit
and indicates the Read (1) or Write (0) operation. The following 7 bits contain the Register Address. In cases of multiple byte Read/Writes, data is two or more bytes:
*/

/**
 * @brief Definition for connected to SPI1 (APB2 PCLK = 84MHz)
 */
#define ICM20602_SPI_CHANNEL		SPI1

#define ICM20602_SPI_SCLK_PIN		LL_GPIO_PIN_5
#define ICM20602_SPI_SCLK_PORT		GPIOA
#define ICM20602_SPI_SCLK_CLK		LL_AHB1_GRP1_PERIPH_GPIOA

#define ICM20602_SPI_MISO_PIN		LL_GPIO_PIN_6
#define ICM20602_SPI_MISO_PORT		GPIOA
#define ICM20602_SPI_MISO_CLK		LL_AHB1_GRP1_PERIPH_GPIOA

#define ICM20602_SPI_MOSI_PIN		LL_GPIO_PIN_7
#define ICM20602_SPI_MOSI_PORT		GPIOA
#define ICM20602_SPI_MOSI_CLK		LL_AHB1_GRP1_PERIPH_GPIOA

#define ICM20602_SPI_CS_PIN			LL_GPIO_PIN_4
#define ICM20602_SPI_CS_PORT		GPIOC
#define ICM20602_SPI_CS_CLK			LL_AHB1_GRP1_PERIPH_GPIOC

#define ICM20602_INT_PIN			LL_GPIO_PIN_5
#define ICM20602_INT_PORT			GPIOC
#define ICM20602_INT_CLK			LL_AHB1_GRP1_PERIPH_GPIOC

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define CHIP_SELECT(ICM20602)		LL_GPIO_ResetOutputPin(ICM20602_SPI_CS_PORT, ICM20602_SPI_CS_PIN)
#define CHIP_DESELECT(ICM20602)		LL_GPIO_SetOutputPin(ICM20602_SPI_CS_PORT, ICM20602_SPI_CS_PIN)
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ICM-20602 Register Map
 */

#define	XG_OFFS_TC_H	0x04
#define	XG_OFFS_TC_L	0x05
#define	YG_OFFS_TC_H	0x07
#define	YG_OFFS_TC_L	0x08
#define	ZG_OFFS_TC_H	0x0A
#define	ZG_OFFS_TC_L	0x0B
#define	SELF_TEST_X_ACCEL	0x0D
#define	SELF_TEST_Y_ACCEL	0x0E
#define	SELF_TEST_Z_ACCEL	0x0F
#define	XG_OFFS_USRH	0x13
#define	XG_OFFS_USRL	0x14
#define	YG_OFFS_USRH	0x15
#define	YG_OFFS_USRL	0x16
#define	ZG_OFFS_USRH	0x17
#define	ZG_OFFS_USRL	0x18
#define	SMPLRT_DIV	0x19
#define	CONFIG	0x1A    //The default value of the register is 0x80.
#define	GYRO_CONFIG	0x1B
#define	ACCEL_CONFIG	0x1C
#define	ACCEL_CONFIG2	0x1D
#define	LP_MODE_CFG	0x1E
#define	ACCEL_WOM_X_THR	0x20
#define	ACCEL_WOM_Y_THR	0x21
#define	ACCEL_WOM_Z_THR	0x22
#define	FIFO_EN	0x23
#define	FSYNC_INT	0x36
#define	INT_PIN_CFG	0x37
#define	INT_ENABLE	0x38
#define	FIFO_WM_INT_STATUS	0x39
		
#define	INT_STATUS	0x3A
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H	0x41
#define	TEMP_OUT_L	0x42
#define	GYRO_XOUT_H	0x43
#define	GYRO_XOUT_L	0x44
#define	GYRO_YOUT_H	0x45
#define	GYRO_YOUT_L	0x46
#define	GYRO_ZOUT_H	0x47
#define	GYRO_ZOUT_L	0x48
#define	SELF_TEST_X_GYRO	0x50
#define	SELF_TEST_Y_GYRO	0x51
#define	SELF_TEST_Z_GYRO	0x52
#define	FIFO_WM_TH1	0x60
#define	FIFO_WM_TH2	0x61
#define	SIGNAL_PATH_RESET	0x68
#define	ACCEL_INTEL_CTRL	0x69
#define	USER_CTRL	0x6A
#define	PWR_MGMT_1	0x6B //The default value of the register is 0x41.
#define	PWR_MGMT_2	0x6C
#define	I2C_IF	0x70
#define	FIFO_COUNTH	0x72
#define	FIFO_COUNTL	0x73
#define	FIFO_R_W	0x74
#define	WHO_AM_I	0x75 //The default value of the register is 0x12.
#define	XA_OFFSET_H	0x77
#define	XA_OFFSET_L	0x78
#define	YA_OFFSET_H	0x7A
#define	YA_OFFSET_L	0x7B
#define	ZA_OFFSET_H	0x7D
#define	ZA_OFFSET_L	0x7E


/**
 * @brief ICM20602 structure definition.
 */

typedef struct _ICM20602{
	short acc_x_raw;
	short acc_y_raw;
	short acc_z_raw;
	short temperature_raw;
	short gyro_x_raw;
	short gyro_y_raw;
	short gyro_z_raw;
	
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
}Struct_ICM20602;

/**
 * @brief ICM20602 structure definition.
 */

extern Struct_ICM20602 ICM20602;
extern int32_t gyro_x_offset, gyro_y_offset, gyro_z_offset;

/**
 * @brief ICM20602 function prototype definition.
 */

void ICM20602_GPIO_SPI_Initialization(void);
int ICM20602_Initialization(void);
void ICM20602_Get6AxisRawData(short* accel, short* gyro);
void ICM20602_Get3AxisGyroRawData(short* gyro);
void ICM20602_Get3AxisAccRawData(short* accel);
int ICM20602_DataReady(void);

#endif
