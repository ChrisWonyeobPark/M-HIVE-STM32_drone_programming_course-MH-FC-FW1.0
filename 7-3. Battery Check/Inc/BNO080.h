/*
  This is a library written for the BNO080
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14686

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The BNO080 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO080 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * This library source code has been modified for STM32F4. Only supports SPI.
 *
 * Development environment specifics:
 * STM32CubeIDE 1.0.0
 * STM32CubeF4 FW V1.24.1
 * STM32F4 LL Driver(SPI) and HAL Driver(RCC for HAL_Delay() function)
 *
 * Modified by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, June, 2019
 * Rev. 1.0
 *
 * https://github.com/ChrisWonyeobPark/BNO080-STM32F4-SPI-LL-Driver
 */

#ifndef	_BNO080_H
#define	_BNO080_H

#include "main.h"
//////////////////////////////////////////////////////////////////////////

/**
 * @brief Definition for connected to SPI2 (APB1 PCLK = 42MHz)
 */
#define BNO080_SPI_CHANNEL		SPI2

#define BNO080_SPI_SCLK_PIN		LL_GPIO_PIN_13
#define BNO080_SPI_SCLK_PORT	GPIOB
#define BNO080_SPI_SCLK_CLK		LL_AHB1_GRP1_PERIPH_GPIOB

#define BNO080_SPI_MISO_PIN		LL_GPIO_PIN_14
#define BNO080_SPI_MISO_PORT	GPIOB
#define BNO080_SPI_MISO_CLK		LL_AHB1_GRP1_PERIPH_GPIOB

#define BNO080_SPI_MOSI_PIN		LL_GPIO_PIN_15
#define BNO080_SPI_MOSI_PORT	GPIOB
#define BNO080_SPI_MOSI_CLK		LL_AHB1_GRP1_PERIPH_GPIOB

#define BNO080_SPI_CS_PIN		LL_GPIO_PIN_12
#define BNO080_SPI_CS_PORT		GPIOB
#define BNO080_SPI_CS_CLK		LL_AHB1_GRP1_PERIPH_GPIOB

#define BNO080_PS0_WAKE_PIN		LL_GPIO_PIN_8
#define BNO080_PS0_WAKE_PORT	GPIOA
#define BNO080_PS0_WAKE_CLK		LL_AHB1_GRP1_PERIPH_GPIOA

#define BNO080_RST_PIN			LL_GPIO_PIN_9
#define BNO080_RST_PORT			GPIOC
#define BNO080_RST_CLK			LL_AHB1_GRP1_PERIPH_GPIOC

#define BNO080_INT_PIN			LL_GPIO_PIN_8
#define BNO080_INT_PORT			GPIOC
#define BNO080_INT_CLK			LL_AHB1_GRP1_PERIPH_GPIOC

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define CHIP_SELECT(BNO080)		LL_GPIO_ResetOutputPin(BNO080_SPI_CS_PORT, BNO080_SPI_CS_PIN)
#define CHIP_DESELECT(BNO080)	LL_GPIO_SetOutputPin(BNO080_SPI_CS_PORT, BNO080_SPI_CS_PIN)

#define WAKE_HIGH()				LL_GPIO_SetOutputPin(BNO080_PS0_WAKE_PORT, BNO080_PS0_WAKE_PIN)
#define WAKE_LOW()				LL_GPIO_ResetOutputPin(BNO080_PS0_WAKE_PORT, BNO080_PS0_WAKE_PIN)

#define RESET_HIGH()			LL_GPIO_SetOutputPin(BNO080_RST_PORT, BNO080_RST_PIN)
#define RESET_LOW()				LL_GPIO_ResetOutputPin(BNO080_RST_PORT, BNO080_RST_PIN)
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//Registers
enum Registers
{
	CHANNEL_COMMAND = 0,
	CHANNEL_EXECUTABLE = 1,
	CHANNEL_CONTROL = 2,
	CHANNEL_REPORTS = 3,
	CHANNEL_WAKE_REPORTS = 4,
	CHANNEL_GYRO = 5
};

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

void BNO080_GPIO_SPI_Initialization(void);
int BNO080_Initialization(void);
unsigned char SPI2_SendByte(unsigned char data);

int BNO080_dataAvailable(void);
void BNO080_parseCommandReport(void);
void BNO080_parseInputReport(void);

float BNO080_getQuatI();
float BNO080_getQuatJ();
float BNO080_getQuatK();
float BNO080_getQuatReal();
float BNO080_getQuatRadianAccuracy();
uint8_t BNO080_getQuatAccuracy();
float BNO080_getAccelX();
float BNO080_getAccelY();
float BNO080_getAccelZ();
uint8_t BNO080_getAccelAccuracy();
float BNO080_getLinAccelX();
float BNO080_getLinAccelY();
float BNO080_getLinAccelZ();
uint8_t BNO080_getLinAccelAccuracy();
float BNO080_getGyroX();
float BNO080_getGyroY();
float BNO080_getGyroZ();
uint8_t BNO080_getGyroAccuracy();
float BNO080_getMagX();
float BNO080_getMagY();
float BNO080_getMagZ();
uint8_t BNO080_getMagAccuracy();
uint16_t BNO080_getStepCount();
uint8_t BNO080_getStabilityClassifier();
uint8_t BNO080_getActivityClassifier();
uint32_t BNO080_getTimeStamp();
int16_t BNO080_getQ1(uint16_t recordID);
int16_t BNO080_getQ2(uint16_t recordID);
int16_t BNO080_getQ3(uint16_t recordID);
float BNO080_getResolution(uint16_t recordID);
float BNO080_getRange(uint16_t recordID);

uint32_t BNO080_readFRSword(uint16_t recordID, uint8_t wordNumber);
void BNO080_frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
int BNO080_readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);
void BNO080_softReset(void);
uint8_t BNO080_resetReason();

float BNO080_qToFloat(int16_t fixedPointValue, uint8_t qPoint);

void BNO080_enableRotationVector(uint16_t timeBetweenReports);
void BNO080_enableGameRotationVector(uint16_t timeBetweenReports);
void BNO080_enableAccelerometer(uint16_t timeBetweenReports);
void BNO080_enableLinearAccelerometer(uint16_t timeBetweenReports);
void BNO080_enableGyro(uint16_t timeBetweenReports);
void BNO080_enableMagnetometer(uint16_t timeBetweenReports);
void BNO080_enableStepCounter(uint16_t timeBetweenReports);
void BNO080_enableStabilityClassifier(uint16_t timeBetweenReports);

void BNO080_calibrateAccelerometer();
void BNO080_calibrateGyro();
void BNO080_calibrateMagnetometer();
void BNO080_calibratePlanarAccelerometer();
void BNO080_calibrateAll();
void BNO080_endCalibration();
int BNO080_calibrationComplete();

void BNO080_setFeatureCommand(uint8_t reportID, uint32_t microsBetweenReports, uint32_t specificConfig);
void BNO080_sendCommand(uint8_t command);
void BNO080_sendCalibrateCommand(uint8_t thingToCalibrate);
void BNO080_requestCalibrationStatus();
void BNO080_saveCalibration();

int BNO080_waitForSPI(void);
int BNO080_receivePacket(void);
int BNO080_sendPacket(uint8_t channelNumber, uint8_t dataLength);

#endif
