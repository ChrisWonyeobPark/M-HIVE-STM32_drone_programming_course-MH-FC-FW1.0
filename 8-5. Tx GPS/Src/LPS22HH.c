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

#include "LPS22HH.h"
#include <math.h>

Struct_LPS22HH LPS22HH;


void LPS22HH_GPIO_SPI_Initialization(void)
{
	LL_SPI_InitTypeDef SPI_InitStruct = {0};
	
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	/**SPI3 GPIO Configuration
	PB3   ------> SPI3_SCK
	PB4   ------> SPI3_MISO
	PB5   ------> SPI3_MOSI
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 10;
	LL_SPI_Init(LPS22HH_SPI_CHANNEL, &SPI_InitStruct);
	LL_SPI_SetStandard(LPS22HH_SPI_CHANNEL, LL_SPI_PROTOCOL_MOTOROLA);

	/**LPS22HH GPIO Control Configuration
	 * PB6  ------> LPS22HH_SPI_CS_PIN (output)
	 * PB7  ------> LPS22HH_INT_PIN (input)
	 */
	/**/
	LL_GPIO_ResetOutputPin(LPS22HH_SPI_CS_PORT, LPS22HH_SPI_CS_PIN);
	
	/**/
	GPIO_InitStruct.Pin = LPS22HH_SPI_CS_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(LPS22HH_SPI_CS_PORT, &GPIO_InitStruct);
	
	/**/
	GPIO_InitStruct.Pin = LPS22HH_INT_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(LPS22HH_INT_PORT, &GPIO_InitStruct);
	
	LL_SPI_Enable(LPS22HH_SPI_CHANNEL);
	
	CHIP_DESELECT(LPS22HH);
}


unsigned char SPI3_SendByte(unsigned char data)
{
	while(LL_SPI_IsActiveFlag_TXE(LPS22HH_SPI_CHANNEL)==RESET);
	LL_SPI_TransmitData8(LPS22HH_SPI_CHANNEL, data);
	
	while(LL_SPI_IsActiveFlag_RXNE(LPS22HH_SPI_CHANNEL)==RESET);
	return LL_SPI_ReceiveData8(LPS22HH_SPI_CHANNEL);
}

//////////////////////////////////////////////////////////////

uint8_t LPS22HH_Readbyte(uint8_t reg_addr)
{
	uint8_t val;

	CHIP_SELECT(LPS22HH);
	SPI3_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
	val = SPI3_SendByte(0x00); //Send DUMMY
	CHIP_DESELECT(LPS22HH);
	
	return val;
}

void LPS22HH_Readbytes(unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	unsigned int i = 0;

	CHIP_SELECT(LPS22HH);
	SPI3_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
	while(i < len)
	{
		data[i++] = SPI3_SendByte(0x00); //Send DUMMY
	}
	CHIP_DESELECT(LPS22HH);
}

void LPS22HH_Writebyte(uint8_t reg_addr, uint8_t val)
{
	CHIP_SELECT(LPS22HH);
	SPI3_SendByte(reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	SPI3_SendByte(val); //Data
	CHIP_DESELECT(LPS22HH);
}

void LPS22HH_Writebytes(unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	unsigned int i = 0;
	CHIP_SELECT(LPS22HH);
	SPI3_SendByte(reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	while(i < len)
	{
		SPI3_SendByte(data[i++]); //Data
	}
	CHIP_DESELECT(LPS22HH);
}




int LPS22HH_Initialization(void)
{
	uint8_t temp_reg;
	uint8_t who_am_i = 0;
	
	LPS22HH_GPIO_SPI_Initialization();
	
	printf("Checking LPS22HH...");
	
	// check WHO_AM_I (0x0F)
	who_am_i = LPS22HH_Readbyte(0x0F); 

	// who am i = 0xb3
	if( who_am_i == 0xb3)
	{
		printf("\nLPS22HH who_am_i = 0x%02x...OK\n\n", who_am_i );
	}
	// recheck
	else if( who_am_i != 0xb3)
	{
		who_am_i = LPS22HH_Readbyte(0x0F); // check WHO_AM_I (0x0F)

		if ( who_am_i != 0xb3 ){
			printf( "nLPS22HH Not OK: 0x%02x Should be 0x%02x\n", who_am_i, 0xb3);
			return 1; //ERROR
		}
	}
	
	// Reset LPS22HH
	// CTRL_REG2 0x11
	LPS22HH_Writebyte(CTRL_REG2, 0x04);
	//printf("LPS22HH Reset");
	do{
		//printf(".");
	}
	while((LPS22HH_Readbyte(CTRL_REG2) & 0x04) != 0x00);
	//printf("Complete\n");
	
	// Set Output Data Rate
	//0x00: One Shot
	//0x10: 1Hz	0x20: 10Hz	0x30: 25Hz	0x40: 50Hz
	//0x50: 75Hz	0x60: 100Hz	0x70: 200Hz
	temp_reg = LPS22HH_Readbyte(CTRL_REG1);
	temp_reg = temp_reg | 0x40;
	LPS22HH_Writebyte(CTRL_REG1, temp_reg);
	temp_reg = 0;
	temp_reg = LPS22HH_Readbyte(CTRL_REG1);
	//printf("%x\n", temp_reg);
	
	// Enable LPF, Cut-off frequency
	//0x08: ODR/9	0x0c: ODR/20
	temp_reg = LPS22HH_Readbyte(CTRL_REG1);
	temp_reg = temp_reg | 0x0c;
	LPS22HH_Writebyte(CTRL_REG1, temp_reg);
	
	// Enable Block Data Update
	temp_reg = LPS22HH_Readbyte(CTRL_REG1);
	temp_reg = temp_reg | 0x02;
	LPS22HH_Writebyte(CTRL_REG1, temp_reg);
	
	// Enable Low Noise Mode (ODR should be lower than 100Hz. This is igonored when ODR = 100Hz or 200Hz)
	temp_reg = LPS22HH_Readbyte(CTRL_REG2);
	temp_reg = temp_reg | 0x02;
	LPS22HH_Writebyte(CTRL_REG2, temp_reg);
	
	// Enable Data-ready signal on INT-DRDY pin
	temp_reg = LPS22HH_Readbyte(CTRL_REG3);
	temp_reg = temp_reg | 0x04;
	LPS22HH_Writebyte(CTRL_REG3, temp_reg);
	
	return 0; //OK
}


int LPS22HH_DataReady(void)
{
	return LL_GPIO_IsInputPinSet(LPS22HH_INT_PORT, LPS22HH_INT_PIN);
}

void LPS22HH_GetPressure(int32_t* pressure)
{
	LPS22HH_Readbytes(PRESSURE_OUT_XL, 3, (unsigned char*)pressure);
}

void LPS22HH_GetTemperature(int16_t* temperature)
{
	LPS22HH_Readbytes(TEMP_OUT_L, 2, (unsigned char*)temperature);
}

#define SEA_PRESSURE 1013.25f

float getAltitude1(float pressure) //No temperature correction.
{
	return (powf((SEA_PRESSURE / pressure), 0.1902226f) - 1.0) * 44307.69396f; //145366.45f * 0.3048f = 44307.69396f;
}

float getAltitude2(float pressure, float temperature) //Get Altitude with temperature correction.
{
	return ((powf((SEA_PRESSURE / pressure), 0.1902226f) - 1.0f) * (temperature + 273.15f)) / 0.0065f;
}
