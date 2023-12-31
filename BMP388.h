/*
 * BMP388.h
 *
 *  Created on: Dec 10, 2022
 *      Author: sam
 */

#ifndef BMP388_H_
#define BMP388_H_


/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"


/* Declarations and definitions ----------------------------------------------*/
#define BMP388_ADDR 				0x76

#define BMP388_CHIP_ID				0x50

// BMP 388 commands
#define BMP388_CMD_RDY				0x10
#define BMP388_SOFTRESET			0xB6

// Over sampling macros
#define BMP388_NO_OVERSAMPLING		0b00000000
#define BMP388_OVERSAMPLING_2X      0b00000001
#define BMP388_OVERSAMPLING_4X      0b00000010
#define BMP388_OVERSAMPLING_8X      0b00000011
#define BMP388_OVERSAMPLING_16X     0b00000100
#define BMP388_OVERSAMPLING_32X     0b00000101

// Filter setting macros
#define BMP3_IIR_FILTER_DISABLE     0b00000000
#define BMP3_IIR_FILTER_COEFF_1     0b00000001
#define BMP3_IIR_FILTER_COEFF_3     0b00000010
#define BMP3_IIR_FILTER_COEFF_7     0b00000011
#define BMP3_IIR_FILTER_COEFF_15    0b00000100
#define BMP3_IIR_FILTER_COEFF_31    0b00000101
#define BMP3_IIR_FILTER_COEFF_63    0b00000110
#define BMP3_IIR_FILTER_COEFF_127   0b00000111

// output data rate macros
#define BMP3_ODR_200_HZ             0b00000000
#define BMP3_ODR_100_HZ             0b00000001
#define BMP3_ODR_50_HZ              0b00000010
#define BMP3_ODR_25_HZ              0b00000011
#define BMP3_ODR_12_5_HZ            0b00000100
#define BMP3_ODR_6_25_HZ            0b00000101
#define BMP3_ODR_3_1_HZ             0b00000110
#define BMP3_ODR_1_5_HZ             0b00000111
#define BMP3_ODR_0_78_HZ            0b00001000
#define BMP3_ODR_0_39_HZ            0b00001001
#define BMP3_ODR_0_2_HZ             0b00001010
#define BMP3_ODR_0_1_HZ             0b00001011
#define BMP3_ODR_0_05_HZ            0b00001100
#define BMP3_ODR_0_02_HZ            0b00001101
#define BMP3_ODR_0_01_HZ            0b00001110
#define BMP3_ODR_0_006_HZ           0b00001111
#define BMP3_ODR_0_003_HZ           0b00010000
#define BMP3_ODR_0_001_HZ           0b00010001

#define BMP388_CALIBDATA_LEN	21



typedef struct{
	float		par_t1;
	float		par_t2;
	float		par_t3;
	float		par_p1;
	float		par_p2;
	float		par_p3;
	float		par_p4;
	float		par_p5;
	float		par_p6;
	float		par_p7;
	float		par_p8;
	float		par_p9;
	float		par_p10;
	float		par_p11;
//	float 		t_lin;
}Calib_data;

// BMP388 registers
typedef enum{
	CHIP_ID					= 0x00,
	ERR_REG					= 0x02,
	STATUS					= 0x03,
	DATA_0					= 0x04, // PRESS_XLSB_7_0
	DATA_1					= 0x05, // PRESS_LSB_15_8
	DATA_2			 		= 0x06, // PRESS_MSB_23_16
	DATA_3					= 0x07, // TEMP_XLSB_7_0
	DATA_4					= 0x08, // TEMP_LSB_15_8
	DATA_5					= 0x09, // TEMP_MSB_23_16
	SENSORTIME_0			= 0x0C,
	SENSORTIME_1			= 0x0D,
	SENSORTIME_2			= 0x0E,
	EVENT					= 0x10,
	INT_STATUS				= 0x11,
	FIFO_LENGTH_0			= 0x12,
	FIFO_LENGTH_1			= 0x13,
	FIFO_DATA				= 0x14,
	FIFO_WTM_0				= 0x15,
	FIFO_WTM_1				= 0x16,
	FIFO_CONFIG_1			= 0x17,
	FIFO_CONFIG_2			= 0x18,
	INT_CTRL				= 0x19,
	IF_CONF					= 0x1A,
	PWR_CTRL				= 0x1B,
	OSR						= 0x1C,
	ODR						= 0x1D,
	CONFIG					= 0x1F,
	CALIB_DATA				= 0x31,
	CMD						= 0x7E,
}BMP388_regs;



class BMP388{
public:
	/* Functions -----------------------------------------------------------------*/
	BMP388(I2C_HandleTypeDef *hi2c_ptr){
		_hi2c_ptr = hi2c_ptr;
	};

	HAL_StatusTypeDef init();
	HAL_StatusTypeDef readData(float *pressure, float *temperature);
	float getPress();
	float findAltitude(float sea_level);
	HAL_StatusTypeDef setTempOS(uint8_t oversample);
	HAL_StatusTypeDef setPressOS(uint8_t oversample);
	HAL_StatusTypeDef setIIRFilterCoeff(uint8_t filtercoeff);
	HAL_StatusTypeDef setOutputDataRate(uint8_t odr);

private:
	/* Functions -----------------------------------------------------------------*/
	HAL_StatusTypeDef softReset();
	HAL_StatusTypeDef getCalibData();
	float compensatePressure();
	float compensateTemp();
	HAL_StatusTypeDef readBytes(BMP388_regs reg_addr, uint8_t *buff, uint8_t len);
	HAL_StatusTypeDef writeBytes(BMP388_regs reg_addr, uint8_t *buff, uint8_t len);

	/* Variables -----------------------------------------------------------------*/
	I2C_HandleTypeDef 		*_hi2c_ptr;

	float					_press;
	float					_temp;
	uint32_t				_raw_press;
	uint32_t				_raw_temp;

	uint8_t					_oversampling;
	uint8_t					_filtercoeff;
	uint8_t					_odr;

	Calib_data				_calib_data;
};

#endif /* BMP388_H_ */
