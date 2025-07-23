/**
 ******************************************************************************
 * @file       TMAG5273.h
 * @brief      Header for TMAG5273 file.
 * @brief      Handle the TMAG5273 Low-Power Linear 3D Hall-Effect Sensor
 *             I2C Interface for STM32 HAL environment
 ******************************************************************************
 * @author     Andreas
 *
 * @date 05. jan. 2023 Created
 *
 ******************************************************************************
 */

#ifndef TMAG5273_H
#define TMAG5273_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Exported macro ------------------------------------------------------------*/

// Error Codes
#define TMAG_CRC_ERROR 2
#define TMAG_ERR 1
#define TMAG_OK 0

// Registers
#define DEVICE_CONFIG_1 0x00
#define DEVICE_CONFIG_2 0x01
#define SENSOR_CONFIG_1 0x02
#define SENSOR_CONFIG_2 0x03
#define X_THR_CONFIG 0x04
#define Y_THR_CONFIG 0x05
#define Z_THR_CONFIG 0x06
#define T_CONFIG 0x07
#define INT_CONFIG_1 0x08
#define MAG_GAIN_CONFIG 0x09
#define MAG_OFFSET_CONFIG_1 0x0A
#define MAG_OFFSET_CONFIG_2 0x0B
#define I2C_ADDRESS 0x0C
#define DEVICE_ID 0x0D
#define MANUFACTURER_ID_LSB 0x0E
#define MANUFACTURER_ID_MSB 0x0F
#define T_MSB_RESULT 0x10
#define T_LSB_RESULT 0x11
#define X_MSB_RESULT 0x12
#define X_LSB_RESULT 0x13
#define Y_MSB_RESULT 0x14
#define Y_LSB_RESULT 0x15
#define Z_MSB_RESULT 0x16
#define Z_LSB_RESULT 0x17
#define CONV_STATUS 0x18
#define ANGLE_RESULT_MSB 0x19
#define ANGLE_RESULT_LSB 0x1A
#define MAGNITUDE_RESULT 0x1B
#define DEVICE_STATUS 0x1C

#define REG_INT_DIS_CONFIG_1 (0x01 | (0x3 << 2))
#define REG_INT_ENA_CONFIG_1 (0x2 << 2)

#define REG_DEVICE_CONFIG_1_DEFAULT 0x00
#define REG_DEVICE_CONFIG_2_DEFAULT 0x00
#define REG_SENSOR_CONFIG_1_DEFAULT 0//0x74
#define REG_SENSOR_CONFIG_2_DEFAULT 0x03
#define REG_X_THR_CONFIG_DEFAULT 0x00
#define REG_Y_THR_CONFIG_DEFAULT 0x00
#define REG_Z_THR_CONFIG_DEFAULT 0x00
#define REG_T_CONFIG_DEFAULT 0x00
#define REG_INT_CONFIG_1_DEFAULT 0x00
#define REG_MAG_GAIN_CONFIG_DEFAULT 0x00
#define REG_MAG_OFFSET_CONFIG_1_DEFAULT 0x00
#define REG_MAG_OFFSET_CONFIG_2_DEFAULT 0x00

#define TMAG5273_DEFAULT_ADDR 0x35

/* Exported types ------------------------------------------------------------*/

typedef enum TMAG5273_Operating_mode {
	TMAG5273_OPERATING_MODE_STANDBY = 0x0,
	TMAG5273_OPERATING_MODE_SLEEP,
	TMAG5273_OPERATING_MODE_MEASURE,
	TMAG5273_OPERATING_MODE_WS
} TMAG5273_Operating_mode_t;

typedef enum TMAG5273_Read_mode {
	TMAG5273_READ_MODE_STANDARD = 0x0,
	TMAG5273_READ_MODE_SENSOR16,
	TMAG5273_READ_MODE_SENSOR8
} TMAG5273_Read_mode_t;

typedef enum TMAG5273_Mag_range {
	TMAG5273_MAG_RANGE_40MT_133MT = 0x0,
	TMAG5273_MAG_RANGE_80MT_266MT,
	TMAG5273_MAG_RANGE_SIZE
} TMAG5273_Mag_range_t;

typedef enum TMAG5273_X_Y_range {
	TMAG5273_X_Y_RANGE_40MT_133MT = TMAG5273_MAG_RANGE_40MT_133MT,
	TMAG5273_X_Y_RANGE_80MT_266MT = (TMAG5273_MAG_RANGE_80MT_266MT << 1)
} TMAG5273_X_Y_range_t;

typedef enum TMAG5273_Angle_en {
	TMAG5273_ANG_DISABLE = 0,
	TMAG5273_ANG_X_Y = 0x1 << 2,
	TMAG5273_ANG_Y_Z = 0x2 << 2,
	TMAG5273_ANG_X_Z = 0x3 << 2
} TMAG5273_Angle_en_t;

typedef enum TMAG5273_CRC {
	TMAG5273_CRC_DISABLE = 0, TMAG5273_CRC_EN = 0x1 << 7,
} TMAG5273_CRC_t;

typedef enum TMAG5273_Dev {
	TMAG5273_DEV_40MT_80MT = 0x1, TMAG5273_DEV_133MT_266MT, TMAG5273_DEV_ID_SIZE
} TMAG5273_Dev_t;

typedef enum TMAG5273_Lp_ln_mode {
	TMAG5273_LOW_ACTIVE_CURRENT = 0x0, TMAG5273_LOW_NOISE = 0x1 << 4
} TMAG5273_Lp_ln_mode_t;

typedef enum TMAG5273_Mag_tempco_mode {
	TMAG5273_NO_MAG_TEMPCO = 0x0,
	TMAG5273_MAG_TEMPCO_NdBFe = 0x1 << 5,
	TMAG5273_MAG_TEMPCO_FERRITE = 0x3 << 5
} TMAG5273_Mag_tempco_mode_t;

typedef enum TMAG5273_Conv_avg_mode {
	TMAG5273_CONV_AVG_1X = 0x0,
	TMAG5273_CONV_AVG_2X = 0x1 << 2,
	TMAG5273_CONV_AVG_4X = 0x2 << 2,
	TMAG5273_CONV_AVG_8X = 0x3 << 2,
	TMAG5273_CONV_AVG_16X = 0x4 << 2,
	TMAG5273_CONV_AVG_32X = 0x5 << 2,
} TMAG5273_Conv_avg_mode_t;

typedef enum TMAG5273_Temp_ch_en {
	TMAG5273_TEMP_CH_DISABLED = 0x0, TMAG5273_TEMP_CH_ENABLED
} TMAG5273_Temp_ch_en_t;

typedef enum TMAG5273_Mag_ch_en {
	TMAG5276_MAG_All_OFF = 0x0 << 4, // All magnetic channels of off, DEFAULT
	TMAG5276_MAG_X = 0x1 << 4,   // X channel enabled
	TMAG5276_MAG_Y = 0x2 << 4,   // Y channel enabled
	TMAG5276_MAG_X_Y = 0x3 << 4,   // X, Y channel enabled
	TMAG5276_MAG_Z = 0x4 << 4,   // Z channel enabled
	TMAG5276_MAG_Z_X = 0x5 << 4,   // Z, X channel enabled
	TMAG5276_MAG_Y_Z = 0x6 << 4,   // Y, Z channel enabled
	TMAG5276_MAG_X_Y_Z = 0x7 << 4,   // X, Y, Z channel enabled
	TMAG5276_MAG_X_Y_X = 0x8 << 4,   // XYX channel enabled
	TMAG5276_MAG_Y_X_Y = 0x9 << 4,   // YXY channel enabled
	TMAG5276_MAG_Y_Z_Y = 0xA << 4,   // YZY channel enabled
	TMAG5276_MAG_X_Z_X = 0xB << 4    // XZX channel enabled
} TMAG5273_Mag_ch_en_t;

// low power mode between conversions
typedef enum TMAG5273_Sleep {
	TMAG5276_1MS = 0x0,
	TMAG5276_5MS = 0x1,
	TMAG5276_10MS = 0x2,
	TMAG5276_15MS = 0x3,
	TMAG5276_20MS = 0x4,
	TMAG5276_30MS = 0x5,
	TMAG5276_50MS = 0x6,
	TMAG5276_100MS = 0x7,
	TMAG5276_500MS = 0x8,
	TMAG5276_1000MS = 0x9,
	TMAG5276_2000MS = 0xA,
	TMAG5276_5000MS = 0xB,
	TMAG5276_20000MS = 0xC
} TMAG5273_Sleep_t;

// Interrupt mode select.
typedef enum TMAG5273_INT_Mode {
	TMAG5276_NO_IRQ = 0x0,
	TMAG5276_IRQ_INT = (0x1 << 2),
	TMAG5276_IRQ_INT_EX_I2C_BUSY = (0x2 << 2),
	TMAG5276_IRQ_SCL = (0x3 << 2),
	TMAG5276_IRQ_SCL_EX_I2C_BUSY = (0x4 << 2),
} TMAG5273_INT_Mode_t;

typedef struct TMAG5273_INT_Port {
	GPIO_TypeDef *port;
	uint16_t pin;
} TMAG5273_INT_Port_t;

typedef struct TMAG5273_Axis {
	float Bx;
	float By;
	float Bz;
} TMAG5273_Axis_t;

typedef struct TMAG5273_Sensor_Data {
	TMAG5273_Axis_t axis;
	float temperature;
	uint8_t status;
	float angle;
	float magnitude;
} TMAG5273_Sensor_Data_t;

typedef struct TMAG5273_Angle {
	float angle;
	float magnitude;
} TMAG5273_Angle_t;

typedef struct TMAG5273_Handle {
	//IO config
	I2C_HandleTypeDef *pI2c;
	TMAG5273_INT_Port_t intPort;

	//I2C_ADDRESS
	int8_t address;

	//DEVICE_CONFIG_1
	TMAG5273_CRC_t crcEna;
	TMAG5273_Mag_tempco_mode_t magTempcoMode;
	TMAG5273_Conv_avg_mode_t convAvgMode;

	//DEVICE_CONFIG_2
	TMAG5273_Lp_ln_mode_t lplnMode;
	TMAG5273_Read_mode_t readMode;
	TMAG5273_Operating_mode_t operatingMode;

	//SENSOR_CONFIG_1
	TMAG5273_Mag_ch_en_t magChEn;
	TMAG5273_Sleep_t sleep;

	//SENSOR_CONFIG_2
	TMAG5273_Angle_en_t angEn;
	TMAG5273_Mag_range_t magZRange;
	TMAG5273_Mag_range_t magXYRange;

	//T_CONFIG
	TMAG5273_Temp_ch_en_t tempChEn;

	//DEVICE_ID
	uint8_t deviceId;

	//MANUFACTURER_ID (LSB - MSB)
	uint16_t manufactorerId;

	//T_RESULT, X_RESULT, Y_RESULT, Z_RESULT, ANGLE_RESULT and MAGNITUDE_RESULT
	int16_t raw[6];

	//LAST CONV_STATUS 
	uint8_t conv_status;

	//LAST DEVICE_STATUS  
	uint8_t dev_status;

	uint32_t timeout;
	uint16_t estiConversionTime;

	//SENSOR ID (mapping for client identification usage)
	int8_t sensor_id;

	//READ MODE FUNCTION PTR
	void *readFuncPtr;

	uint8_t triggerBit;

	uint8_t lastCrc;

} TMAG5273_Handle_t;

/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void TMAG5273_Init(TMAG5273_Handle_t *pHandle);

void TMAG5273_ModifyI2CAddress(TMAG5273_Handle_t *pHandle, uint8_t new_addr);
int8_t TMAG5273_SearchI2CAddress(TMAG5273_Handle_t *pHandle);

void TMAG5273_EnableTrigger(TMAG5273_Handle_t *pHandle, uint8_t ena);

uint32_t TMAG5273_TriggerConversion(TMAG5273_Handle_t *pHandle);

uint8_t TMAG5273_GetNumOfChEnabled(TMAG5273_Handle_t *pHandle);

float TMAG5273_ReadTemperature(TMAG5273_Handle_t *pHandle);
uint8_t TMAG5273_ReadMagneticField(TMAG5273_Handle_t *pHandle,
		TMAG5273_Axis_t *axis);
uint8_t TMAG5273_ReadAngle(TMAG5273_Handle_t *pHandle, TMAG5273_Angle_t *angle);
uint8_t TMAG5273_ReadSensorData(TMAG5273_Handle_t *pHandle,
		TMAG5273_Sensor_Data_t *sens);

// config functions
void TMAG5273_ConfigOperatingMode(TMAG5273_Handle_t *pHandle,
		TMAG5273_Operating_mode_t mode);
void TMAG5273_ConfigReadMode(TMAG5273_Handle_t *pHandle,
		TMAG5273_Read_mode_t mode);
void TMAG5273_ConfigZMagRange(TMAG5273_Handle_t *pHandle,
		TMAG5273_Mag_range_t range);
void TMAG5273_ConfigXYMagRange(TMAG5273_Handle_t *pHandle,
		TMAG5273_X_Y_range_t range);
void TMAG5273_ConfigLplnMode(TMAG5273_Handle_t *pHandle,
		TMAG5273_Lp_ln_mode_t mode);
void TMAG5273_ConfigMagTempcoMode(TMAG5273_Handle_t *pHandle,
		TMAG5273_Mag_tempco_mode_t mode);
void TMAG5273_ConfigConvAvgMode(TMAG5273_Handle_t *pHandle,
		TMAG5273_Conv_avg_mode_t mode);
void TMAG5273_ConfigCrc(TMAG5273_Handle_t *pHandle, TMAG5273_CRC_t crc);
void TMAG5273_ConfigTempChEnabled(TMAG5273_Handle_t *pHandle, uint8_t enabled);

uint8_t TMAG5273_ReadRegister(TMAG5273_Handle_t *pHandle, uint8_t reg,
		uint8_t length, uint8_t *data);

//inline uint8_t TMAG5273_calculateCRC(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* TMAG5273_H */
