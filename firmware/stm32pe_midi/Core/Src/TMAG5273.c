/**
 ******************************************************************************
 * @file       TMAG5273.c
 * @brief      Source TMAG5273 file.
 * @brief      Handle the TMAG5273 Low-Power Linear 3D Hall-Effect Sensor
 *             I2C Interface for STM32 HAL environment
 *             TODO
 *                  - 7.5.1.3.6 I2C Read CRC - Issues - missing crc
 *                  - Utilize the readFuncPtr in:
 *                             TMAG5273_ReadTemperature
 *                             TMAG5273_ReadMagneticField
 *                             TMAG5273_ReadAngle
 *                             TMAG5273_ReadSensorData
 *
 ******************************************************************************
 * @author     Andreas
 *
 * @date 05. jan. 2023 Created
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "TMAG5273.h"
#include "stdbool.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/

// Ref /doc/tmag5273.pdf -  6.11 Power up & Conversion Time
#define T_START_MEASURE 70   // set to zero seems ok
//(2) Add 25�s for each additional magnetic channel enabled for conversion with
//    CONV_AVG = 000b. When CONV_AVG = 000b, the conversion time doesn't change
//    with the T_CH_EN bit setting.
//(3) For conversion with CONV_AVG =101b, each channel data is collected 32 times.
//    If an additional channel is enabled with CONV_AVG =101b, add 32�25�s = 800�s
//    to the tmeasure to calculate the conversion time for two channels.
#define T_MEASURE_CHANNEL 25
#define T_MEASURE_DUMMY 0

//TODO Verify estimated conversion time
#define CONV_TIME_FROM_AVG_MODE(HANDLE) (T_START_MEASURE + (HANDLE->tempChEn + 3) * T_MEASURE_CHANNEL * (1 + (1 << (HANDLE->convAvgMode >> 2))) + T_MEASURE_DUMMY)

//Ref /doc/tmag5273.pdf - 6.6 Temperature Sensor
#define TSENSET0 25.0
#define TADCRES 60.1
#define TADCT0 17508.0

#define MAG_RANGE_40MT 40.f
#define MAG_RANGE_80MT 80.f
#define MAG_RANGE_133MT 133.f
#define MAG_RANGE_266MT 266.f

//! Byte swap short
#define SWAP_INT16( val )  ((val << 8) | ((val >> 8) & 0xFF))

#define ARRAY_SINGLE_RC_DELAY_MS 1000
#define ARRAY_POWER_UP_TIMEOUT_MS 500

#define CONVERSION_DATA_COMPLETE 0x1

//Conversion BIT (ref doc/TMAG5273.pdf 7.5.1.3.1 and 7.5.1.3.2)
//Applies when I2C_RD is 0 in DEVICE_CONFIG_1, TRIGGER_MODE  is 0 in DEVICE_CONFIG_2
// and OPERATING_MODE is 0 in (Stand-by mode)
#define CONV_TRIGGER_BIT (uint8_t)0x80

#define TMAG5273_CRC_START_8 0xFF

/* Private typedef -----------------------------------------------------------*/

typedef uint8_t (*TMAG5273_ReadFunc_t)(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t length, uint8_t *data);
void atan2CORDIC(int16_t numerator, int16_t denominator,
		int16_t iteration_length, int32_t results[]);

/* Private variables ---------------------------------------------------------*/

// These two arrays are used with the CORDIC function to convert the integer output
// into floating point values
const uint32_t magArray[16] = { 1518500250, 1358187913, 1317635818, 1307460871,
		1304914694, 1304277995, 1304118810, 1304079014, 1304069065, 1304066577,
		1304065955, 1304065800, 1304065761, 1304065751, 1304065749, 1304065748 };

const uint32_t atanArray32[16] = { 536870912, 316933406, 167458907, 85004756,
		42667331, 21354465, 10679838, 5340245, 2670163, 1335087, 667544, 333772,
		166886, 83443, 41722, 20861 };

/*
 * Magnetic resolution in mT for TMAG5273 versions.
 * Scaled = Range[mT] * 1e6 / 2^15 * 10, (1 mT = 10 Gauss)
 */
static const float TMAG5273_RANGE[TMAG5273_DEV_ID_SIZE][TMAG5273_MAG_RANGE_SIZE] =
		{ { 0, 0 }, { 40.f, 80.f }, { 133.f, 266.f } };

/* Private function ----------------------------------------------------------*/
inline uint8_t TMAG5273_ReadSensorRaw(TMAG5273_Handle_t *pHandle);
//inline uint8_t TMAG5273_CRC_8( const uint8_t * input_str, uint16_t num_bytes );

inline uint8_t TMAG5273_verifyCRC(const uint8_t *data, uint8_t len);

inline uint8_t TMAG5273_ReadRegisterStd(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t len, uint8_t *data);
inline uint8_t TMAG5273_ReadRegister16(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t len, uint8_t *data);
inline uint8_t TMAG5273_ReadRegister8(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t len, uint8_t *data);
inline uint8_t TMAG5273_WriteRegister(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t data);
inline uint8_t TMAG5273_WriteRegisters(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t *pData, uint8_t len);
inline uint8_t TMAG5273_WriteRegisterGeneral(TMAG5273_Handle_t *pHandle,
		uint8_t reg, uint8_t data);

#define DEBUG2_LED_GPIO_Port GPIOA
#define DEBUG2_LED_Pin GPIO_PIN_9








/* Global functions ----------------------------------------------------------*/

/**
 * @brief TMAG5273 initializer
 * @param   pHandle - TMAG5273 handler
 * @retval None
 */
void TMAG5273_Init(TMAG5273_Handle_t *pHandle) {
	uint8_t val;
	uint8_t config[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	TMAG5273_CRC_t crc_ena = pHandle->crcEna;

	pHandle->lastCrc = 0;

	//Disable crc check until the CRC register has been configured
	pHandle->crcEna = TMAG5273_CRC_DISABLE;

	//Set timeout for blocking transitions
	if (!pHandle->timeout)
		pHandle->timeout = 1000;

	pHandle->deviceId = 0;
	pHandle->manufactorerId = 0;

	// Read Device ID (and Verify Connection)
	if (TMAG5273_ReadRegisterStd(pHandle, DEVICE_ID, 1, &pHandle->deviceId)) {
		int8_t adr = TMAG5273_SearchI2CAddress(pHandle);
		if (!adr) {
			//Error_Handler();
			while (1) {
				printf("error 1\r\n");
				HAL_GPIO_TogglePin(DEBUG2_LED_GPIO_Port, DEBUG2_LED_Pin);
				HAL_Delay(100);
			}
		}

		//Set the device address
		pHandle->address = adr;
		if (TMAG5273_ReadRegisterStd(pHandle, DEVICE_ID, 1,
				&pHandle->deviceId)) {
			//Error_Handler();
			while (1) {
				printf("error 2\r\n");
				HAL_GPIO_TogglePin(DEBUG2_LED_GPIO_Port, DEBUG2_LED_Pin);
				HAL_Delay(1000);
			}
		}
	}

	if (TMAG5273_ReadRegisterStd(pHandle, MANUFACTURER_ID_LSB, 1,
			(uint8_t*) &pHandle->manufactorerId)) {
		//Error_Handler();
		while (1) {
			printf("error 3\r\n");
			HAL_GPIO_TogglePin(DEBUG2_LED_GPIO_Port, DEBUG2_LED_Pin);
			HAL_Delay(5000);
		}
	}

	if (TMAG5273_ReadRegisterStd(pHandle, MANUFACTURER_ID_MSB, 1, &val)) {
		//Error_Handler();
		while (1) {
			printf("error 4\r\n");
			HAL_GPIO_TogglePin(DEBUG2_LED_GPIO_Port, DEBUG2_LED_Pin);
			HAL_Delay(1000);
		}
	}

	//Expected Value 0x5449 (ref 7.6.1.15, 7.6.1.16 /doc/tmag5273.pdf)
	pHandle->manufactorerId |= (val << 8);

	pHandle->crcEna = crc_ena;

	switch (pHandle->readMode) {
	case TMAG5273_READ_MODE_STANDARD:
		pHandle->readFuncPtr = (void*) TMAG5273_ReadRegisterStd;
		break;
	case TMAG5273_READ_MODE_SENSOR16:
		pHandle->readFuncPtr = (void*) TMAG5273_ReadRegister16;
		break;
	case TMAG5273_READ_MODE_SENSOR8:
		pHandle->readFuncPtr = (void*) TMAG5273_ReadRegister8;
		break;
	default:
		pHandle->readFuncPtr = (void*) TMAG5273_ReadRegisterStd;
	}

	// Configure device
	config[DEVICE_CONFIG_1] = REG_DEVICE_CONFIG_1_DEFAULT
			| pHandle->magTempcoMode | pHandle->convAvgMode | pHandle->readMode
			| pHandle->crcEna;
	config[DEVICE_CONFIG_2] = (REG_DEVICE_CONFIG_2_DEFAULT & 0xEC)
			| pHandle->lplnMode | pHandle->operatingMode;
	config[SENSOR_CONFIG_1] = ((uint8_t) pHandle->magChEn
			| (uint8_t) pHandle->sleep);
	config[SENSOR_CONFIG_2] = (REG_SENSOR_CONFIG_2_DEFAULT & 0xFC)
			| pHandle->angEn | (pHandle->magXYRange << 0x01)
			| pHandle->magZRange;
	config[X_THR_CONFIG] = REG_X_THR_CONFIG_DEFAULT;
	config[Y_THR_CONFIG] = REG_Y_THR_CONFIG_DEFAULT;
	config[Z_THR_CONFIG] = REG_Z_THR_CONFIG_DEFAULT;
	config[T_CONFIG] = (REG_T_CONFIG_DEFAULT & 0xFE) | pHandle->tempChEn;
	config[INT_CONFIG_1] = REG_INT_CONFIG_1_DEFAULT
			| ((!pHandle->intPort.port) ?
					REG_INT_DIS_CONFIG_1 : REG_INT_ENA_CONFIG_1);
	config[MAG_GAIN_CONFIG] = REG_MAG_GAIN_CONFIG_DEFAULT;
	config[MAG_OFFSET_CONFIG_1] = REG_MAG_OFFSET_CONFIG_1_DEFAULT;
	config[MAG_OFFSET_CONFIG_2] = REG_MAG_OFFSET_CONFIG_2_DEFAULT;

	TMAG5273_WriteRegisters(pHandle, DEVICE_CONFIG_1, config,
			MAG_OFFSET_CONFIG_2 + 1);

	pHandle->estiConversionTime = CONV_TIME_FROM_AVG_MODE(pHandle);

}

/**
 * @brief TMAG5273_ModifyI2CAddress - Change device Address
 * @param   pHandle - TMAG5273 handler
 * @param   uint8_t - new address
 * @retval None
 */
void TMAG5273_ModifyI2CAddress(TMAG5273_Handle_t *pHandle, uint8_t new_addr) {
	if (!TMAG5273_WriteRegister(pHandle, I2C_ADDRESS, (new_addr << 1) | 0x01))
		pHandle->address = new_addr;
}

/**
 * @brief TMAG5273_SearchI2CAddress - Find address(7.6.1.13 I2C_ADDRESS Register)
 * @param   pHandle - TMAG5273 handler
 * @retval int8_t - address found (0 if no address found)
 */
int8_t TMAG5273_SearchI2CAddress(TMAG5273_Handle_t *pHandle) {
	int8_t ret = 0;
	int8_t currentDeviceAddress = pHandle->address;
	while (TMAG5273_ReadRegisterStd(pHandle, I2C_ADDRESS, 1, (uint8_t*) &ret)) {
		HAL_Delay(2);
		pHandle->address++;
		if (pHandle->address == currentDeviceAddress)
			break;
	}
	pHandle->address = currentDeviceAddress;
	if (ret) {
		//bit 7-1 I2C_ADDRESS and bit 0 I2C_ADDRESS_UPDATE_EN
		ret = (ret >> 1);
	}
	return ret;
}

/**
 * @brief TMAG5273_EnableTrigger - Trigger adc reading on read back
 * @param   pHandle - TMAG5273 handler
 * @retval void
 */

void TMAG5273_EnableTrigger(TMAG5273_Handle_t *pHandle, uint8_t ena) {
	pHandle->triggerBit = ena;
}

/**
 * @brief TMAG5273_TriggerConversion - Trigger adc reading (standby mode)
 * //TODO altarnativly trigger with the INT pin (gpio push poll)
 * @param   pHandle - TMAG5273 handler
 * @retval number of millisec to wait for conversion to be complete
 */
uint32_t TMAG5273_TriggerConversion(TMAG5273_Handle_t *pHandle) {
	if (pHandle->operatingMode != TMAG5273_OPERATING_MODE_STANDBY)
		return 0;

	//Trigger Conversion
	TMAG5273_WriteRegister(pHandle, DEVICE_ID | CONV_TRIGGER_BIT, 0x00);

	//TODO Find apropriate estimations time
	return pHandle->estiConversionTime;
}

/**
 * @brief TMAG5273_GetNumOfChEnabled - Get The number of channels enabled
 * @param   pHandle - TMAG5273 handler
 * @retval The Number of enable ch
 */
uint8_t TMAG5273_GetNumOfChEnabled(TMAG5273_Handle_t *pHandle) {
	TMAG5273_Mag_ch_en_t magChEn = pHandle->magChEn;
	TMAG5273_Temp_ch_en_t tempChEn = pHandle->tempChEn;
	uint8_t numOfCh = 0;

	if (magChEn > TMAG5276_MAG_X_Y_Z) {
		numOfCh += 2;
	} else {
		if (magChEn & TMAG5276_MAG_Z)
			numOfCh++;

		if (magChEn & TMAG5276_MAG_Y)
			numOfCh++;

		if (magChEn & TMAG5276_MAG_X)
			numOfCh++;
	}

	if (tempChEn)
		numOfCh++;

	return numOfCh;
}

/**
 * @brief TMAG5273_ReadTemperature - Read device temperature
 * @param   pHandle - TMAG5273 handler
 * @retval float - temperature
 */
float TMAG5273_ReadTemperature(TMAG5273_Handle_t *pHandle) {
	int16_t TADCT = 0;
	if (!TMAG5273_ReadRegisterStd(pHandle, T_MSB_RESULT, 2,
			(uint8_t*) &TADCT)) {
		TADCT = SWAP_INT16(TADCT);
		return TSENSET0 + (((float) TADCT - TADCT0) / TADCRES);
	}
	return 0.0;
}

int TMAG5273_map_dev_id(uint8_t raw_id) {
	switch (raw_id) {
	case 0x05:
		return 1; // 40/80mT
	case 0x0A:
		return 2; // 133/266mT
	default:
		return 1;   // fallback to 40/80mT
	}
}

/**
 * @brief TMAG5273_ReadMagneticField - Read Magnetic Fields 7.5.2.1
 * NB - doesn't check conversion status (like TMAG5273_ReadSensorData)
 * @param   pHandle - TMAG5273 handler
 * @param   axis - TMAG5273_Axis_t ptr to axis to update
 * @retval uint8_t - 0 - succsess
 */
uint8_t TMAG5273_ReadMagneticField(TMAG5273_Handle_t *pHandle,
		TMAG5273_Axis_t *axis) {
	uint8_t ret;
	uint8_t msg = (uint8_t) X_MSB_RESULT;

	//printf("deviceId: %d, magXYRange: %d, scale: %f\n", pHandle->deviceId, pHandle->magXYRange, TMAG5273_RANGE[pHandle->deviceId][pHandle->magXYRange]);

	int dev_idx = TMAG5273_map_dev_id(pHandle->deviceId);

	if (pHandle->magChEn == TMAG5276_MAG_All_OFF)
		return TMAG_ERR;

	if (pHandle->readMode == TMAG5273_READ_MODE_STANDARD
			|| pHandle->readMode == TMAG5273_READ_MODE_SENSOR16) {
		msg |= CONV_TRIGGER_BIT;
		if (pHandle->readMode == TMAG5273_READ_MODE_STANDARD) {
			ret = HAL_I2C_Master_Transmit(pHandle->pI2c,
					(pHandle->address << 1), &msg, 1, pHandle->timeout);
			if (!ret)
				ret = HAL_I2C_Master_Receive(pHandle->pI2c,
						(pHandle->address << 1), (uint8_t*) pHandle->raw, 6,
						pHandle->timeout);
		}
	}

	if (!ret) {
		pHandle->raw[0] = SWAP_INT16(pHandle->raw[0]);
		pHandle->raw[1] = SWAP_INT16(pHandle->raw[1]);
		pHandle->raw[2] = SWAP_INT16(pHandle->raw[2]);

		//Ref 7.5.2.1 Magnetic Sensor Data (16 bit eq 10)
		axis->Bx = (((float) pHandle->raw[0]) / 32768.f)
				* TMAG5273_RANGE[dev_idx][pHandle->magXYRange];
		axis->By = (((float) pHandle->raw[1]) / 32768.f)
				* TMAG5273_RANGE[dev_idx][pHandle->magXYRange];
		axis->Bz = (((float) pHandle->raw[2]) / 32768.f)
				* TMAG5273_RANGE[dev_idx][pHandle->magZRange];
	}
	return ret;
}

/**
 * @brief TMAG5273_ReadAngle - Read angle data (and magnitude) Fields 7.5.2.3
 * NB - doesn't check conversion status (like TMAG5273_ReadSensorData)
 * @param   pHandle - TMAG5273 handler
 * @param   axis - TMAG5273_Angle_t ptr to angle dat
 * @retval uint8_t - 0 - succsess
 */
uint8_t TMAG5273_ReadAngle(TMAG5273_Handle_t *pHandle, TMAG5273_Angle_t *angle) {
	uint8_t ret;
	uint8_t val[3];
	int16_t in;
	uint8_t msg = (uint8_t) ANGLE_RESULT_MSB;

	if (pHandle->angEn == TMAG5273_ANG_DISABLE)
		return TMAG_ERR;

	if (pHandle->readMode == TMAG5273_READ_MODE_STANDARD
			|| pHandle->readMode == TMAG5273_READ_MODE_SENSOR16) {
		ret = HAL_I2C_Master_Transmit(pHandle->pI2c, (pHandle->address << 1),
				&msg, 1, pHandle->timeout);
		if (!ret)
			ret = HAL_I2C_Master_Receive(pHandle->pI2c, (pHandle->address << 1),
					val, 3, pHandle->timeout);
	} else {
		//TODO add support for the other modes
		return TMAG_ERR;
	}

	if (!ret) {
		//Ref 7.5.2.3 Angle and Magnitude Data Definition
		in = ((val[0] << 8) | val[1]);
		angle->angle = (float) ((~0xE00F & (uint16_t) in) >> 4)
				+ (float) ((~0xFFF0 & (uint16_t) in) / 16.0);

		angle->magnitude = (((float) ((uint16_t) val[2])) / 8192.f)
				* TMAG5273_RANGE[pHandle->deviceId][pHandle->magXYRange];
	}

	return ret;
}

/**
 * @brief TMAG5273_ReadSensorData - Read all sensor data Fields 7.5.2.1

 * @param   pHandle - TMAG5273 handler
 * @param   axis - TMAG5273_Sensor_Data_t ptr to axis, temperature and statius
 * @retval uint8_t - 0 - succsess
 */
uint8_t TMAG5273_ReadSensorData(TMAG5273_Handle_t *pHandle,
		TMAG5273_Sensor_Data_t *sens) {
	uint8_t ret = 1;

	ret = TMAG5273_ReadSensorRaw(pHandle);
	if (!ret) {
		//Conv Status
		sens->status = pHandle->conv_status;
		if (sens->status & CONVERSION_DATA_COMPLETE) {
			if (pHandle->tempChEn) {
				//Ref 7.5.2.1 Magnetic Sensor Data 16 bits data reading
				sens->temperature = TSENSET0
						+ (((float) pHandle->raw[0] - TADCT0) / TADCRES);
			}

			if (pHandle->magChEn != TMAG5276_MAG_All_OFF) {
				//Ref 7.5.2.1 Magnetic Sensor Data (16 bit eq 10)
				sens->axis.Bx =
						(((float) pHandle->raw[1]) / 32768.f)
								* TMAG5273_RANGE[pHandle->deviceId][pHandle->magXYRange];
				sens->axis.By =
						(((float) pHandle->raw[2]) / 32768.f)
								* TMAG5273_RANGE[pHandle->deviceId][pHandle->magXYRange];
				sens->axis.Bz = (((float) pHandle->raw[3]) / 32768.f)
						* TMAG5273_RANGE[pHandle->deviceId][pHandle->magZRange];
			}

			if (pHandle->angEn != TMAG5273_ANG_DISABLE) {
				//Ref 7.5.2.3 Angle and Magnitude Data Definition
				sens->angle =
						(float) ((~0xE00F & (uint16_t) pHandle->raw[4]) >> 4)
								+ (float) ((~0xFFF0 & (uint16_t) pHandle->raw[4])
										/ 16.0);
				sens->magnitude =
						(((float) ((uint16_t) pHandle->raw[5])) / 8192.f)
								* TMAG5273_RANGE[pHandle->deviceId][pHandle->magXYRange];
			}
		} else
			ret = 1;
	}

	return ret;
}

/**
 * @brief TMAG5273_ConfigOperatingMode - Configure Operating Mode
 * @param   pHandle - TMAG5273 handler
 * @param   mode - TMAG5273_Operating_mode_t
 * @retval void
 */
void TMAG5273_ConfigOperatingMode(TMAG5273_Handle_t *pHandle,
		TMAG5273_Operating_mode_t mode) {
	uint8_t val;
	if (!TMAG5273_ReadRegisterStd(pHandle, DEVICE_CONFIG_2, 1, &val)) {
		if (!TMAG5273_WriteRegister(pHandle, DEVICE_CONFIG_2,
				val | pHandle->lplnMode | mode))
			pHandle->operatingMode = mode;
	}
}

/**
 * @brief TMAG5273_ConfigOperatingMode - Configure Read Mode
 * @param   pHandle - TMAG5273 handler
 * @param   mode - TMAG5273_Read_mode_t
 * @retval void
 */
void TMAG5273_ConfigReadMode(TMAG5273_Handle_t *pHandle,
		TMAG5273_Read_mode_t mode) {
	uint8_t val = (uint8_t) pHandle->magTempcoMode
			| (uint8_t) pHandle->convAvgMode | (uint8_t) mode
			| (uint8_t) pHandle->crcEna;
	if (!TMAG5273_WriteRegister(pHandle, DEVICE_CONFIG_1, val)) {
		pHandle->readMode = mode;

		switch (pHandle->readMode) {
		case TMAG5273_READ_MODE_STANDARD:
			pHandle->readFuncPtr = (void*) TMAG5273_ReadRegisterStd;
			break;
		case TMAG5273_READ_MODE_SENSOR16:
			pHandle->readFuncPtr = (void*) TMAG5273_ReadRegister16;
			break;
		case TMAG5273_READ_MODE_SENSOR8:
			pHandle->readFuncPtr = (void*) TMAG5273_ReadRegister8;
			break;
		default:
			pHandle->readFuncPtr = (void*) TMAG5273_ReadRegisterStd;
		}
	}
}

/**
 * @brief TMAG5273_ConfigZMagRange - Configure Magnetic Range (ref 6.7 - 6.8)
 * @param   pHandle - TMAG5273 handler
 * @param   range - TMAG5273_Mag_range_t
 * @retval void
 */
void TMAG5273_ConfigZMagRange(TMAG5273_Handle_t *pHandle,
		TMAG5273_Mag_range_t range) {
	uint8_t val;
	if (!TMAG5273_ReadRegisterStd(pHandle, SENSOR_CONFIG_2, 1, &val)) {
		if (!TMAG5273_WriteRegister(pHandle, SENSOR_CONFIG_2, val | range)) {
			pHandle->magZRange = range;
		}
	}
}

/**
 * @brief TMAG5273_ConfigXYMagRange - Configure Magnetic Range (ref 6.7 - 6.8)
 * @param   pHandle - TMAG5273 handler
 * @param   range - TMAG5273_Mag_range_t
 * @retval void
 */
void TMAG5273_ConfigXYMagRange(TMAG5273_Handle_t *pHandle,
		TMAG5273_X_Y_range_t range) {
	uint8_t val;
	if (!TMAG5273_ReadRegisterStd(pHandle, SENSOR_CONFIG_2, 1, &val)) {
		if (!TMAG5273_WriteRegister(pHandle, SENSOR_CONFIG_2, val | (range))) {
			pHandle->magXYRange = (TMAG5273_Mag_range_t) (range >> 1);
		}
	}
}

/**
 * @brief TMAG5273_ConfigLplnMode - Configure DC Power Selection (ref 6.5)
 * @param   pHandle - TMAG5273 handler
 * @param   mode - TMAG5273_Lp_ln_mode_t
 * @retval void
 */
void TMAG5273_ConfigLplnMode(TMAG5273_Handle_t *pHandle,
		TMAG5273_Lp_ln_mode_t mode) {
	uint8_t val;
	if (!TMAG5273_ReadRegisterStd(pHandle, DEVICE_CONFIG_2, 1, &val)) {
		if (!TMAG5273_WriteRegister(pHandle, DEVICE_CONFIG_2,
				val | mode | pHandle->operatingMode))
			pHandle->lplnMode = mode;
	}
}

/**
 * @brief TMAG5273_ConfigMagTempcoMode - Configure Magnetic Temperature Compensation (ref 6.9)
 * @param   pHandle - TMAG5273 handler
 * @param   mode - TMAG5273_Mag_tempco_mode_t
 * @retval void
 */
void TMAG5273_ConfigMagTempcoMode(TMAG5273_Handle_t *pHandle,
		TMAG5273_Mag_tempco_mode_t mode) {

	if (!TMAG5273_WriteRegister(pHandle, DEVICE_CONFIG_1,
			(uint8_t) mode | (uint8_t) pHandle->convAvgMode
					| (uint8_t) pHandle->readMode | (uint8_t) pHandle->crcEna))
		pHandle->magTempcoMode = mode;
}

/**
 * @brief TMAG5273_ConfigConvAvgMode - Configure Update Rate Settings (ref 7.3.6)
 * @param   pHandle - TMAG5273 handler
 * @param   mode - TMAG5273_Mag_tempco_mode_t
 * @retval void
 */
void TMAG5273_ConfigConvAvgMode(TMAG5273_Handle_t *pHandle,
		TMAG5273_Conv_avg_mode_t mode) {
	if (!TMAG5273_WriteRegister(pHandle, DEVICE_CONFIG_1,
			(uint8_t) pHandle->magTempcoMode | (uint8_t) mode
					| (uint8_t) pHandle->readMode
					| (uint8_t) pHandle->crcEna)) {
		pHandle->convAvgMode = mode;
		pHandle->estiConversionTime = CONV_TIME_FROM_AVG_MODE(pHandle);
	}
}

/**
 * @brief TMAG5273_ConfigCrc - Configure crc Settings (ref 7.3.6)
 * @param   pHandle - TMAG5273 handler
 * @param   crc - TMAG5273_CRC_t
 * @retval void
 */
void TMAG5273_ConfigCrc(TMAG5273_Handle_t *pHandle, TMAG5273_CRC_t crc) {
	if (!TMAG5273_WriteRegister(pHandle, DEVICE_CONFIG_1,
			(uint8_t) pHandle->magTempcoMode | (uint8_t) pHandle->convAvgMode
					| (uint8_t) pHandle->readMode | (uint8_t) crc)) {
		pHandle->crcEna = crc;
	}
}

/**
 * @brief TMAG5273_ConfigTempChEnabled - Enables data acquisition of the temperature channel (ref 7.6.1.8)
 * @param   pHandle - TMAG5273 handler
 * @param   enable - TMAG5273_ConfigTempChEnabled
 * @retval void
 */
void TMAG5273_ConfigTempChEnabled(TMAG5273_Handle_t *pHandle, uint8_t enabled) {
	TMAG5273_Temp_ch_en_t tempChEn;
	uint8_t val;
	if (enabled)
		tempChEn = TMAG5273_TEMP_CH_ENABLED;
	else
		tempChEn = TMAG5273_TEMP_CH_DISABLED;

	if (!TMAG5273_ReadRegisterStd(pHandle, T_CONFIG, 1, &val)) {
		if (!TMAG5273_WriteRegister(pHandle, T_CONFIG, val | tempChEn)) {
			pHandle->tempChEn = tempChEn;
			pHandle->estiConversionTime = CONV_TIME_FROM_AVG_MODE(pHandle);
		}
	}
}

/**
 * @brief TMAG5273_Read - Read handle (ref 7.5.1.3.3 - 7.5.1.3.5)
 * @param   pHandle - TMAG5273 handler
 * @param   reg_or_length - uint8_t - length (in case of 16 or 8) or register std
 * @retval uint8_t - 0 success
 */
uint8_t TMAG5273_ReadRegister(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t len, uint8_t *data) {
	uint8_t ret;
	TMAG5273_ReadFunc_t pReadFunc = (TMAG5273_ReadFunc_t) pHandle->readFuncPtr;
	TMAG5273_Read_mode_t currentReadMode = pHandle->readMode;

	if ((reg < T_MSB_RESULT)
			&& (currentReadMode != TMAG5273_READ_MODE_STANDARD)) {
		TMAG5273_ConfigReadMode(pHandle, TMAG5273_READ_MODE_STANDARD);
		pReadFunc = (TMAG5273_ReadFunc_t) TMAG5273_ReadRegisterStd;
	}

	ret = pReadFunc(pHandle, reg, len, data);

	if (currentReadMode != pHandle->readMode) {
		TMAG5273_ConfigReadMode(pHandle, currentReadMode);
	}

	return ret;

}

/* Private function ----------------------------------------------------------*/

/**
 * @brief TMAG5273_ReadRegisterStd - Read standard 3 byte I2C value(s) Blocking (ref 7.5.1.3.3)
 * @param   pHandle - TMAG5273 handler
 * @param   reg - uint8_t - Register to read
 * @param   data - uint8_t Ptr to the buffer to update
 * @retval uint8_t - 0 success
 */
uint8_t TMAG5273_ReadRegisterStd(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t len, uint8_t *data) {
	uint8_t ret;
	uint8_t msg[3] = { reg, len, 0 };

	if (pHandle->triggerBit)
		msg[0] |= CONV_TRIGGER_BIT;

	if (pHandle->crcEna) {
		// If CRC is enabled, the secondary will send the fifth CRC byte
		// based off the CRC calculation of immediate past 4 register bytes
		uint8_t ldata[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		uint8_t lSize = 0;
		while (lSize < len) {
			ret = HAL_I2C_Master_Transmit(pHandle->pI2c,
					(pHandle->address << 1), msg, 1, pHandle->timeout);

			if (!ret) {
				ret = HAL_I2C_Master_Receive(pHandle->pI2c,
						(pHandle->address << 1), &ldata[lSize], 5,
						pHandle->timeout);
			}

			if (ret)
				break;

			pHandle->lastCrc = ldata[4];

			if (TMAG5273_verifyCRC(ldata, 4)) {
				//TODO remove copy
				for (uint8_t i = 0; (i < 4) && ((i + lSize) < len); i++)
					data[i + lSize] = ldata[i];
				ret = 1;
				break;
			}

			for (uint8_t i = 0; (i < 4) && ((i + lSize) < len); i++)
				data[i + lSize] = ldata[i];

			// Inc Req and len
			msg[0] = +4;
			lSize += 4;
		}
	} else {
		ret = HAL_I2C_Master_Transmit(pHandle->pI2c, (pHandle->address << 1),
				msg, 1, pHandle->timeout);

		if (!ret) {
			ret = HAL_I2C_Master_Receive(pHandle->pI2c, (pHandle->address << 1),
					data, len, pHandle->timeout);
		}
	}

	return ret;
}

/**
 * @brief TMAG5273_ReadRegister8 - Read 8 bit value Blocking (ref 7.5.1.3.5)
 * @param   pHandle - TMAG5273 handler
 * @param   reg - uint8_t - Register to read
 * @param   data - uint8_t Ptr to the buffer to update
 * @retval uint8_t - 0 success
 */
uint8_t TMAG5273_ReadRegister8(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t len, uint8_t *data) {
	uint8_t ret;

	// Data to read is number of enabled channels + conversion status
	uint8_t rSize = TMAG5273_GetNumOfChEnabled(pHandle) + 1;

	if ((pHandle->readMode != TMAG5273_READ_MODE_SENSOR8)) {
		return TMAG_ERR;
	}

	if (len < rSize)
		return TMAG_ERR;

	if (pHandle->crcEna) {
		uint8_t ldata[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		ret = HAL_I2C_Master_Receive(pHandle->pI2c, (pHandle->address << 1),
				ldata, rSize + 1, pHandle->timeout);

		if (TMAG5273_verifyCRC(ldata, rSize - 1)) {
			ret = TMAG_ERR;
		} else {
			for (uint8_t i = 0; i < rSize; i++)
				data[i] = ldata[i];
		}
	} else {
		ret = HAL_I2C_Master_Receive(pHandle->pI2c, (pHandle->address << 1),
				data, rSize, pHandle->timeout);
	}

	return ret;
}

/**
 * @brief TMAG5273_ReadRegister16 - Read 16 bit value Blocking (ref 7.5.1.3.4)
 * @param   pHandle - TMAG5273 handler
 * @param   len - uint8_t - Registers to read from  CONV_STATUS
 * @param   data - int16_t Ptr to the buffer to update
 * @retval uint8_t - 0 success
 */
uint8_t TMAG5273_ReadRegister16(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t len, uint8_t *data) {
	uint8_t ret;

	// Data to read is number of enabled channels + conversion status
	uint8_t rSize = (TMAG5273_GetNumOfChEnabled(pHandle) * 2) + 1;

	if (pHandle->readMode != TMAG5273_READ_MODE_SENSOR16) {
		return TMAG_ERR;
	}

	if (len < rSize)
		return TMAG_ERR;

	if (pHandle->crcEna) {
		uint8_t ldata[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		ret = HAL_I2C_Master_Receive(pHandle->pI2c, (pHandle->address << 1),
				ldata, rSize + 1, pHandle->timeout);

		if (TMAG5273_verifyCRC(ldata, rSize - 1)) {
			ret = TMAG_ERR;
		} else {
			for (uint8_t i = 0; i < rSize; i++)
				data[i] = ldata[i];
		}
	} else {
		ret = HAL_I2C_Master_Receive(pHandle->pI2c, (pHandle->address << 1),
				data, rSize, pHandle->timeout);
	}

	return ret;
}

/**
 * @brief TMAG5273_WriteRegister - Write standard 8 bit value Blocking (ref 7.5.1.3.1)
 * @param   pHandle - TMAG5273 handler
 * @param   reg - uint8_t - Register to write
 * @param   data - uint8_t data to write
 * @retval uint8_t - 0 success
 */
uint8_t TMAG5273_WriteRegister(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t data) {
	uint8_t msg[2] = { reg, data };
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(pHandle->pI2c,
			(pHandle->address << 1), msg, 2, pHandle->timeout);
	printf("I2C Write: addr=0x%02X reg=0x%02X data=0x%02X status=%d\n",
			pHandle->address, reg, data, status);
	return status;
}

/**
 * @brief TMAG5273_WriteRegisters - Write standard 8 bit value Blocking (ref 7.5.1.3.1)
 * @param   pHandle - TMAG5273 handler
 * @param   reg - uint8_t - Register to write
 * @param   pData - uint8_t * data to write
 * @param   len - uint8_t length og data
 * @retval uint8_t - 0 success
 */
uint8_t TMAG5273_WriteRegisters(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t *pData, uint8_t len) {
	uint8_t msg[20] = { reg };
	uint8_t i;
	if (len > (sizeof(msg) - 1))
		return TMAG_ERR;

	for (i = 1; i < len + 1; i++)
		msg[i] = pData[i - 1];

	return HAL_I2C_Master_Transmit(pHandle->pI2c, (pHandle->address << 1), msg,
			len + 1, pHandle->timeout);
}

/**
 * @brief TMAG5273_WriteRegister - Write general 8 bit value (ref 7.5.1.3.2)
 * @param   pHandle - TMAG5273 handler
 * @param   reg - uint8_t - Register to write
 * @param   data - uint8_t data to write
 * @retval uint8_t - 0 success
 */
uint8_t TMAG5273_WriteRegisterGeneral(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t data) {
	uint8_t msg[10] = { reg, data };
	return HAL_I2C_Master_Transmit(pHandle->pI2c, 0x00, msg, 2,
			pHandle->timeout);
}

/**
 * @brief TMAG5273_ReadSensorRaw - TODO Utilize readFuncPtr 

 * @param   pHandle - TMAG5273 handler
 * @retval uint8_t - 0 - succsess
 */
uint8_t TMAG5273_ReadSensorRaw(TMAG5273_Handle_t *pHandle) {
	uint8_t val[15];
	uint8_t ret = 1;
	uint8_t msg = (uint8_t) T_MSB_RESULT;

	if (pHandle->readMode != TMAG5273_READ_MODE_STANDARD) {
		return TMAG_ERR;
	}

	ret = HAL_I2C_Master_Transmit(pHandle->pI2c, (pHandle->address << 1), &msg,
			1, pHandle->timeout);

	if (!ret) {
		ret = HAL_I2C_Master_Receive(pHandle->pI2c, (pHandle->address << 1),
				val, 13, pHandle->timeout);
	}

	if (!ret) {
		//Conv Status
		pHandle->conv_status = val[8];

		//Device Status
		pHandle->dev_status = val[12];

		if (pHandle->conv_status & CONVERSION_DATA_COMPLETE) {
			pHandle->raw[0] = ((val[0] << 8) | val[1]);
			pHandle->raw[1] = ((val[2] << 8) | val[3]);
			pHandle->raw[2] = ((val[4] << 8) | val[5]);
			pHandle->raw[3] = ((val[6] << 8) | val[7]);
			pHandle->raw[4] = ((val[9] << 8) | val[10]);
			pHandle->raw[5] = (int16_t) val[11];
		} else
			ret = 1;
	}

	return ret;
}

/**
 * @brief  Calculate CRC for I2C data frame
 *               0x00 = Standard 3-byte I2C Read
 *               0x01 = 1-byte I2C Read Command for 16-bit Data
 *               0x02 = 1-byte I2C Read Command for 8-bit Data
 *               NB! Requires a minimum of 1 channel enabled
 *               (NOTE: if using the Standard 3-byte I2C Read, default will be 0x04)
 *               Values if using 1-byte I2C Read Command for 16-/8-bit Data
 *               0x01 = Single Axis Measurement
 *               0x02 = Two Axis Measurement
 *               0x03 = Three Axis Measurement
 *               0x04 = All Sensors Measurement (Not valid for 16-bit Data Read)
 *
 * @param   data     - type of I2C Read being used:
 *               
 *
 * @param   len      - Length of crc calculation
 *
 * Takes in an array containing an I2C data frame (MSB to LSB) with the CRC bits
 * all set to ZERO and calculates and returns the CRC for that data frame.
 * @retval uint8_t - crc
 */
uint8_t TMAG5273_calculateCRC(const uint8_t *data, uint8_t len) {
	int i = 0;
	uint8_t crc = 0xFF;
	uint8_t crcNew = 0x00;
	uint8_t d[8];
	uint8_t c[8];
	int j = 0;

//  if (i2cRead == 0x01)
//    numChannels = numChannels * 2;
//  else if (i2cRead == 0x00)
//    numChannels = 4;

	for (i = 0; i < len; i++) {
		for (j = 0; j < 8; j++) {
			d[j] = (data[i] >> j) & 0x01;
			c[j] = (crc >> j) & 0x01;
		}

		crcNew = d[7] ^ d[6] ^ d[0] ^ c[0] ^ c[6] ^ c[7];
		crcNew |= (d[6] ^ d[1] ^ d[0] ^ c[0] ^ c[1] ^ c[6]) << 1;
		crcNew |= (d[6] ^ d[2] ^ d[1] ^ d[0] ^ c[0] ^ c[1] ^ c[2] ^ c[6]) << 2;
		crcNew |= (d[7] ^ d[3] ^ d[2] ^ d[1] ^ c[1] ^ c[2] ^ c[3] ^ c[7]) << 3;
		crcNew |= (d[4] ^ d[3] ^ d[2] ^ c[2] ^ c[3] ^ c[4]) << 4;
		crcNew |= (d[5] ^ d[4] ^ d[3] ^ c[3] ^ c[4] ^ c[5]) << 5;
		crcNew |= (d[6] ^ d[5] ^ d[4] ^ c[4] ^ c[5] ^ c[6]) << 6;
		crcNew |= (d[7] ^ d[6] ^ d[5] ^ c[5] ^ c[6] ^ c[7]) << 7;
		crc = crcNew;
	}

	return crc;
}

uint8_t TMAG5273_verifyCRC(const uint8_t *data, uint8_t len) {
	uint8_t crc_received = data[len] & 0xFF;
	uint8_t crc_calc = TMAG5273_calculateCRC(data, len);

	return (crc_received == crc_calc) ? TMAG_OK : TMAG_ERR;
}

/**
 // * @brief   Verify CRC in I2C data frame
 // *
 // * @param   i2cRead     - type of I2C Read being used:
 // *               0x00 = Standard 3-byte I2C Read
 // *               0x01 = 1-byte I2C Read Command for 16-bit Data
 // *               0x02 = 1-byte I2C Read Command for 8-bit Data
 // *
 // * @param   numChannels - number of channels to read, requires a minimum of 1 channel
 // *               (NOTE: if using the Standard 3-byte I2C Read, default will be 0x04)
 // *               Values if using 1-byte I2C Read Command for 16-/8-bit Data
 // *               0x01 = Single Axis Measurement
 // *               0x02 = Two Axis Measurement
 // *               0x03 = Three Axis Measurement
 // *               0x04 = All Sensors Measurement (Not valid for 16-bit Data Read)
 // *
 // * Takes in an array containing an I2C data frame (MSB to LSB) and checks if the
 // * CRC bits (according to their locations for the TMAG5x73) are correct according
 // * to the CRC calculation algorithm.
 // * @retval uint8_t - crc
 // */
//uint8_t TMAG5273_verifyCRC(uint8_t i2cRead, uint8_t numChannels, uint8_t data[])
//{
//    if ((i2cRead > 0x02) || (numChannels == 0) || (numChannels > 0x04)) assert(1 == 0);
//
//    uint8_t crc_received = data[4] & 0xFF;
//    data[4] &= ~(0xFF); // the CRC bits of the data must be 0000b to calculate its CRC correctly
//    uint8_t crc_calc = TMAG5x73calculateCRC(i2cRead, numChannels, data);
//    data[4] |= crc_received; // the previously removed CRC bits are reinserted
//
//    return crc_received == crc_calc;
//}
///**
// * @brief   Verify CRC in I2C data frame
// *
// * @param   input_str     const uint8_t ptr to data buffer
// * @param   num_bytes     number of bytes in the data buffer
// * @retval uint8_t - crc
// */
//uint8_t TMAG5273_CRC_8( const uint8_t * input_str, uint16_t num_bytes ) 
//{
//
//	uint16_t a;
//	uint8_t crc;
//	const unsigned char *ptr;
//
//	crc = TMAG5273_CRC_START_8;
//	ptr = input_str;
//
//	if ( ptr != NULL ) for (a=0; a<num_bytes; a++) 
//  {
//
//		crc = TMAG5273_crc_table[(*ptr++) ^ crc];
//	}
//
//	return crc;
//
//}  /* crc_8 */

//****************************************************************************
//! Calculate Angle and Magnitude using CORDIC for two axes
//! Takes in a float array of at least size 2, two magnetic axis measurements, and their
//! shared range and changes indexes 0 and 1 of the array to the calculated angle and
//! magnitude using CORDIC.
//!
//! CORDIC_results[] - float array of at least size 2, will have indexes 0 and 1 replaced
//!                    with the calculated angle and magnitude, respectively
//! numerator - magnetic measurement result pulled from the register of the vertical axis
//! denominator - magnetic measurement result pulled from the register of the horizontal axis
//! range - the range in mT shared by both axes (axes cannot have different set ranges or CORDIC will not
//!         be accurate)
//!
//! For angle measurements to match the in-built CORDIC on the device, match the numerator and denominator
//! for the associated ANGLE_EN value below, while also ensuring both axes share the same range (in mT):
//!
//!              ANGLE_EN value | numerator axis | denominator axis
//!                   0x00             none             none
//!                   0x01              Y                X
//!                   0x02              Z                Y
//!                   0x03              Z                X
//!
//! The returned angle will be interpreted as 0deg on the positive denominator axis and 90deg on the positive numerator axis
//!
//! For more information on CORDIC algorithms please watch the "CORDIC algorithm for angle calculations" video
//! provided by Texas Instruments: https://training.ti.com/cordic-algorithm-angle-calculations
//****************************************************************************
void calcCORDIC(float CORDIC_results[], int16_t numerator, int16_t denominator,
		uint16_t range, int16_t iteration_length) {
	if (iteration_length > 16 || iteration_length < 1)
		return;

	int32_t angle_readings[3];
	int32_t ANGLE_CALC_32;
	int32_t MAG_CALC_CORDIC;

	atan2CORDIC(numerator, denominator, iteration_length, angle_readings);

	ANGLE_CALC_32 = angle_readings[0];
	MAG_CALC_CORDIC = angle_readings[1];

	CORDIC_results[0] = (((float) ANGLE_CALC_32) / 65536) * 360 / 65536; // angle result
	CORDIC_results[1] = ((float) MAG_CALC_CORDIC) * range / 32768; // mag result
}

//****************************************************************************
//! atan2 + magnitude calculation using CORDIC algorithm
//!
//! Implementation of the CORDIC algorithm without result value conversion for
//! faster use with functions repeatedly using the CORDIC algorithm (see the
//! planeAngles function).
//!
//! numerator - magnetic measurement result pulled from the register of the vertical axis
//! denominator - magnetic measurement result pulled from the register of the horizontal axis
//! iteration_length - the number of "rotations" to be made in the calculation, more generally
//!                    means a more accurate calculation (max amount is 16)
//! results - int32_t array of at lease size 2. results[0] will store the unconverted angle
//!           value calculated by the algorithm. results[1] will store the unconverted
//!           magnitude value. See the calcCORDIC function for how to convert these values
//!           using the lookup tables included.
//!
//! For more information on CORDIC algorithms please watch the "CORDIC algorithm for angle calculations" video
//! provided by Texas Instruments: https://training.ti.com/cordic-algorithm-angle-calculations
//****************************************************************************
void atan2CORDIC(int16_t numerator, int16_t denominator,
		int16_t iteration_length, int32_t results[]) {
	if (iteration_length > 16 || iteration_length < 1)
		return;

	int i = 0;
	int32_t num_old, den_old, num, den;
	uint32_t angle;
	num = num_old = numerator;
	den = den_old = denominator;

	if (den < 0)
		angle = 0x80000000;
	else
		angle = 0;

	for (i = 0; i < iteration_length; i++) {

		if (((den >= 0) && (num < 0)) || ((den < 0) && (num >= 0))) {
			den = den - (num_old >> i);
			num = num + (den_old >> i);
			angle = angle - atanArray32[i];
		} else {
			den = den + (num_old >> i);
			num = num - (den_old >> i);
			angle = angle + atanArray32[i];
		}
		den_old = den;
		num_old = num;
	}
	results[0] = angle;
	if (den < 0)
		den = -den;
	results[1] = (((int64_t) den) * magArray[i - 1]) >> (15 + 16);
}

uint8_t TMAG5273_RewriteI2CAddress(TMAG5273_Handle_t *pHandle, uint8_t new_addr) {
    if (new_addr < 0x08 || new_addr > 0x77)
        return 3;
    uint8_t reg_val = (new_addr << 1) | 0x01;
    uint8_t result = TMAG5273_WriteRegister(pHandle, 0x0C, reg_val);
    if (result == 0) {
        pHandle->address = new_addr; // update handle's address
    }
    return result;
}
