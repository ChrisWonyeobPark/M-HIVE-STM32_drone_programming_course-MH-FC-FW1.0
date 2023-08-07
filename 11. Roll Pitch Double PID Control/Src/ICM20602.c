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

/**
 * @brief ICM20602 structure definition.
 */

#include "ICM20602.h"

Struct_ICM20602 ICM20602;
int32_t gyro_x_offset, gyro_y_offset, gyro_z_offset; // To remove offset


void ICM20602_GPIO_SPI_Initialization(void)
{
	LL_SPI_InitTypeDef SPI_InitStruct = {0};
	
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	/**SPI1 GPIO Configuration
	PA5   ------> SPI1_SCK
	PA6   ------> SPI1_MISO
	PA7   ------> SPI1_MOSI
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8; //ICM-20602 MAX SPI CLK is 10MHz. But DIV2(42MHz) is available.
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 10;
	LL_SPI_Init(ICM20602_SPI_CHANNEL, &SPI_InitStruct);
	LL_SPI_SetStandard(ICM20602_SPI_CHANNEL, LL_SPI_PROTOCOL_MOTOROLA);
	
	/**ICM20602 GPIO Control Configuration
	 * PC4  ------> ICM20602_SPI_CS_PIN (output)
	 * PC5  ------> ICM20602_INT_PIN (input)
	 */
	/**/
	LL_GPIO_ResetOutputPin(ICM20602_SPI_CS_PORT, ICM20602_SPI_CS_PIN);
	
	/**/
	GPIO_InitStruct.Pin = ICM20602_SPI_CS_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(ICM20602_SPI_CS_PORT, &GPIO_InitStruct);
	
	/**/
	GPIO_InitStruct.Pin = ICM20602_INT_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(ICM20602_INT_PORT, &GPIO_InitStruct);

	LL_SPI_Enable(ICM20602_SPI_CHANNEL);

	CHIP_DESELECT(ICM20602);
}


unsigned char SPI1_SendByte(unsigned char data)
{
	while(LL_SPI_IsActiveFlag_TXE(ICM20602_SPI_CHANNEL)==RESET);
	LL_SPI_TransmitData8(ICM20602_SPI_CHANNEL, data);
	
	while(LL_SPI_IsActiveFlag_RXNE(ICM20602_SPI_CHANNEL)==RESET);
	return LL_SPI_ReceiveData8(ICM20602_SPI_CHANNEL);
}

//////////////////////////////////////////////////////////////

uint8_t ICM20602_Readbyte(uint8_t reg_addr)
{
	uint8_t val;

	CHIP_SELECT(ICM20602);
	SPI1_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
	val = SPI1_SendByte(0x00); //Send DUMMY to read data
	CHIP_DESELECT(ICM20602);
	
	return val;
}

void ICM20602_Readbytes(unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	unsigned int i = 0;

	CHIP_SELECT(ICM20602);
	SPI1_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
	while(i < len)
	{
		data[i++] = SPI1_SendByte(0x00); //Send DUMMY to read data
	}
	CHIP_DESELECT(ICM20602);
}

void ICM20602_Writebyte(uint8_t reg_addr, uint8_t val)
{
	CHIP_SELECT(ICM20602);
	SPI1_SendByte(reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	SPI1_SendByte(val); //Send Data to write
	CHIP_DESELECT(ICM20602);
}

void ICM20602_Writebytes(unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	unsigned int i = 0;
	CHIP_SELECT(ICM20602);
	SPI1_SendByte(reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	while(i < len)
	{
		SPI1_SendByte(data[i++]); //Send Data to write
	}
	CHIP_DESELECT(ICM20602);
}


int ICM20602_Initialization(void)
{

	uint8_t who_am_i = 0;
	int16_t accel_raw_data[3] = {0};  // To remove offset
	int16_t gyro_raw_data[3] = {0};   // To remove offset
	
	ICM20602_GPIO_SPI_Initialization();
	
	printf("Checking ICM20602...");
	
	// check WHO_AM_I (0x75)
	who_am_i = ICM20602_Readbyte(WHO_AM_I); 

	// who am i = 0x12
	if(who_am_i == 0x12)
	{
		printf("\nICM20602 who_am_i = 0x%02x...OK\n\n", who_am_i);
	}
	// recheck
	else if(who_am_i != 0x12)
	{
		who_am_i = ICM20602_Readbyte(WHO_AM_I); // check again WHO_AM_I (0x75)

		if (who_am_i != 0x12){
			printf( "ICM20602 Not OK: 0x%02x Should be 0x%02x\n", who_am_i, 0x12);
			return 1; //ERROR
		}
	}
	
	// Reset ICM20602
	// PWR_MGMT_1 0x6B
	ICM20602_Writebyte(PWR_MGMT_1, 0x80); //Reset ICM20602
	HAL_Delay(50);

	// PWR_MGMT_1 0x6B
	ICM20602_Writebyte(PWR_MGMT_1, 0x01); // Enable Temperature sensor(bit4-0), Use PLL(bit2:0-01)
									// 온도센서 끄면 자이로 값 이상하게 출력됨
	HAL_Delay(50);

	// PWR_MGMT_2 0x6C
	ICM20602_Writebyte(PWR_MGMT_2, 0x38); // Disable Acc(bit5:3-111), Enable Gyro(bit2:0-000)
	//ICM20602_Writebyte( PWR_MGMT_2, 0x00 ); // Enable Acc(bit5:3-000), Enable Gyro(bit2:0-000)
	HAL_Delay(50);
	
	// set sample rate to 1000Hz and apply a software filter
	ICM20602_Writebyte(SMPLRT_DIV, 0x00);
	HAL_Delay(50);
	
	// Gyro DLPF Config
	//ICM20602_Writebyte(CONFIG, 0x00); // Gyro LPF fc 250Hz(bit2:0-000)
	ICM20602_Writebyte(CONFIG, 0x05); // Gyro LPF fc 20Hz(bit2:0-100) at 1kHz sample rate
	HAL_Delay(50);

	// GYRO_CONFIG 0x1B
	ICM20602_Writebyte(GYRO_CONFIG, 0x18); // Gyro sensitivity 2000 dps(bit4:3-11), FCHOICE (bit1:0-00)
	HAL_Delay(50);

	// ACCEL_CONFIG 0x1C
	ICM20602_Writebyte(ACCEL_CONFIG, 0x18); // Acc sensitivity 16g
	HAL_Delay(50);
	
	// ACCEL_CONFIG2 0x1D
	ICM20602_Writebyte(ACCEL_CONFIG2, 0x03); // Acc FCHOICE 1kHz(bit3-0), DLPF fc 44.8Hz(bit2:0-011)
	HAL_Delay(50);
	
	// Enable Interrupts when data is ready
	ICM20602_Writebyte(INT_ENABLE, 0x01); // Enable DRDY Interrupt
	HAL_Delay(50);
	
	//printf("gyro bias: %d %d %d\n", gyro_x_offset, gyro_y_offset, gyro_z_offset);
	
	// Remove Gyro X offset
//	ICM20602_Writebyte( XG_OFFS_USRH, offset_x>>8 );	// gyro x offset high byte
//	ICM20602_Writebyte( XG_OFFS_USRL, offset_x );	// gyro x offset low byte
//	
//	// Remove Gyro Y offset
//	ICM20602_Writebyte( YG_OFFS_USRH, offset_y>>8 );	// gyro y offset high byte
//	ICM20602_Writebyte( YG_OFFS_USRL, offset_y );	// gyro y offset low byte
//	
//	// Remove Gyro Z offset
//	ICM20602_Writebyte( ZG_OFFS_USRH, offset_z>>8 );	// gyro z offset high byte
//	ICM20602_Writebyte( ZG_OFFS_USRL, offset_z );	// gyro z offset low byte

	return 0; //OK
}

void ICM20602_Get6AxisRawData(short* accel, short* gyro)
{
	unsigned char data[14];
	ICM20602_Readbytes(ACCEL_XOUT_H, 14, data);
	
	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];

	gyro[0] = ((data[8] << 8) | data[9]);
	gyro[1] = ((data[10] << 8) | data[11]);
	gyro[2] = ((data[12] << 8) | data[13]);
}

void ICM20602_Get3AxisGyroRawData(short* gyro)
{
	unsigned char data[6];
	ICM20602_Readbytes(GYRO_XOUT_H, 6, data);
	
	gyro[0] = ((data[0] << 8) | data[1]);
	gyro[1] = ((data[2] << 8) | data[3]);
	gyro[2] = ((data[4] << 8) | data[5]);
}

void ICM20602_Get3AxisAccRawData(short* accel)
{
	unsigned char data[6];
	ICM20602_Readbytes(ACCEL_XOUT_H, 6, data);
	
	accel[0] = ((data[0] << 8) | data[1]);
	accel[1] = ((data[2] << 8) | data[3]);
	accel[2] = ((data[4] << 8) | data[5]);
}

int ICM20602_DataReady(void)
{
	return LL_GPIO_IsInputPinSet(ICM20602_INT_PORT, ICM20602_INT_PIN);
}
