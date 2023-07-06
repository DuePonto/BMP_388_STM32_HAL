/*
 * BMP388.cpp
 *
 *  Created on: Dec 10, 2022
 *      Author: sam
 */

#include "BMP388.h"
#include "math.h"




/*!
 *  @brief Function to initiate BMP388 and get calibration data
 *
 *  @param none
 *
 *  @return Status of execution
 *  @retval = HAL_OK  		-> Success
 *  @retval != HAL_OK	  	-> Failure Info
 *
 */
HAL_StatusTypeDef BMP388::init(){
	HAL_StatusTypeDef rslt;
	uint8_t chip_id;

	// Read CHIP_ID byte
	rslt = readBytes(CHIP_ID, &chip_id, 1);
	if(rslt == HAL_OK && chip_id == BMP388_CHIP_ID){
		// using softreset command
		rslt = softReset();
		if(rslt == HAL_OK){
			// get calibration data
			rslt = getCalibData();
		}
		else{
			return rslt;
		}
	}
	else{
		return rslt;
	}

	return rslt;
}




/*!
 *  @brief Function to read pressure and temperature from BMP388
 *
 *  @param[out] pressure	: Pointer to the variable that contain pressure.
 *	@param[out] temperature	: Pointer to the variable that contain temperature.
 *
 *  @return Status of execution
 *  @retval = HAL_OK  		-> Success
 *  @retval != HAL_OK	  	-> Failure Info
 *
 */
HAL_StatusTypeDef BMP388::readData(float *pressure, float *temperature){
	HAL_StatusTypeDef rslt;
	uint8_t power_mode = 0b00100011;

	// Set OSR register
	rslt = writeBytes(OSR, &_oversampling, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set ODR register
	rslt = writeBytes(ODR, &_odr, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set CONFIG register
	rslt = writeBytes(CONFIG, &_filtercoeff, 1);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Set PWR_CTRL register
	rslt = writeBytes(PWR_CTRL, &power_mode, 1);
	if(rslt != HAL_OK){
		return rslt;
	}

	uint8_t raw_data[6];
	// Get raw data for pressure and temperature
	rslt = readBytes(DATA_0, raw_data, 6);
	if(rslt != HAL_OK){
		return rslt;
	}
	// Temporary variables to store the sensor data
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	// Parsing pressure data
	data_xlsb = (uint32_t)raw_data[0];
	data_lsb = (uint32_t)raw_data[1] << 8;
	data_msb = (uint32_t)raw_data[2] << 16;
	_raw_press = data_msb | data_lsb | data_xlsb;

	// Parsing temperature data
	data_xlsb = (uint32_t)raw_data[3];
	data_lsb = (uint32_t)raw_data[4] << 8;
	data_msb = (uint32_t)raw_data[5] << 16;
	_raw_temp = data_msb | data_lsb | data_xlsb;

	compensateTemp();
	compensatePressure();

	*pressure = _press;
	*temperature = _temp;

	return rslt;
}




float BMP388::getPress(){
	return _press;
}




float BMP388::findAltitude(float sea_level){
	  // Equation taken from BMP180 datasheet (page 16):
	  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	  // Note that using the equation from wikipedia can give bad results
	  // at high altitude. See this thread for more information:
	  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	  return 44330.0 * (1.0 - pow(_press / sea_level, 0.1903));
}




HAL_StatusTypeDef BMP388::setTempOS(uint8_t oversample){
	if(oversample > BMP388_OVERSAMPLING_32X){
		return HAL_ERROR;
	}
	_oversampling = (_oversampling & 0b11000111) | (oversample << 3);
	return HAL_OK;
}




HAL_StatusTypeDef BMP388::setPressOS(uint8_t oversample){
	if(oversample > BMP388_OVERSAMPLING_32X){
		return HAL_ERROR;
	}
	_oversampling = (_oversampling & 0b11111000) | oversample;
	return HAL_OK;
}




HAL_StatusTypeDef BMP388::setIIRFilterCoeff(uint8_t filtercoeff){
	if(filtercoeff > BMP3_IIR_FILTER_COEFF_127){
		return HAL_ERROR;
	}
	_filtercoeff = filtercoeff;
	return HAL_OK;
}




HAL_StatusTypeDef BMP388::setOutputDataRate(uint8_t odr){
	if(odr > BMP3_ODR_0_001_HZ){
		return HAL_ERROR;
	}
	_odr = odr;
	return HAL_OK;
}




/*!
 *  @brief Function to send softreset cmnd
 *
 *  @param none
 *
 *  @return Status of execution
 *  @retval = HAL_OK  		-> Success
 *  @retval != HAL_OK	  	-> Failure Info
 *
 */
HAL_StatusTypeDef BMP388::softReset(){
	uint8_t rst_cmnd = BMP388_SOFTRESET;
    uint8_t cmd_rdy_status;
    uint8_t cmd_err_status;

	HAL_StatusTypeDef rslt;

	// Читаем статус регистр, чтобы понять готов ли BMP принять команду
	rslt = readBytes(STATUS, &cmd_rdy_status, 1);
	if((rslt == HAL_OK) && (cmd_rdy_status & BMP388_CMD_RDY)){
		// После проверки пишем в регистр CMD команду перезагрузки
		rslt = writeBytes(CMD, &rst_cmnd, 1);
		if(rslt == HAL_OK){
			// После проверки пауза на 2 мс и чтение регистра ошибки с проверкой
			HAL_Delay(2);
			rslt = readBytes(ERR_REG, &cmd_err_status, 1);
			if((cmd_err_status & CMD) || (rslt != HAL_OK)){
				return rslt;
			}
		}
		else{
			return rslt;
		}
	}

	return rslt;
}




/*!
 *  @brief Function to get calibration data
 *
 *  @param none
 *
 *  @return Status of execution
 *  @retval = HAL_OK  		-> Success
 *  @retval != HAL_OK	  	-> Failure Info
 *
 */
HAL_StatusTypeDef BMP388::getCalibData(){
	HAL_StatusTypeDef rslt;
	uint8_t calib_buff[BMP388_CALIBDATA_LEN] = {0};

	uint16_t	raw_par_t1;
	uint16_t	raw_par_t2;
	int8_t		raw_par_t3;
	int16_t		raw_par_p1;
	int16_t		raw_par_p2;
	int8_t		raw_par_p3;
	int8_t		raw_par_p4;
	uint16_t	raw_par_p5;
	uint16_t	raw_par_p6;
	int8_t		raw_par_p7;
	int8_t		raw_par_p8;
	int16_t		raw_par_p9;
	int8_t		raw_par_p10;
	int8_t		raw_par_p11;

	rslt = readBytes(CALIB_DATA, calib_buff, BMP388_CALIBDATA_LEN);

	float temp_var;
	if(rslt == HAL_OK){
		// PAR_T1
		temp_var = 0.00390625f;
		raw_par_t1 = ((uint16_t)calib_buff[1] << 8) | (uint16_t)calib_buff[0];
		_calib_data.par_t1 = (float)raw_par_t1 / temp_var;
		// PAR_T2
		temp_var = 1073741824.f;
		raw_par_t2 = ((uint16_t)calib_buff[3] << 8) | (uint16_t)calib_buff[2];
		_calib_data.par_t2 = (float)raw_par_t2 / temp_var;
		// PAR_T3
		temp_var = 281474976710656.f;
		raw_par_t3 = calib_buff[4];
		_calib_data.par_t3 = (float)raw_par_t3 / temp_var;
		// PAR_P1
		temp_var = 1048576.f;
		raw_par_p1 = ((int16_t)calib_buff[6] << 8) | (int16_t)calib_buff[5];
		_calib_data.par_p1 = ((float)raw_par_p1 - 16384) / temp_var;
		// PAR_P2
		temp_var = 536870912.f;
		raw_par_p2 = ((int16_t)calib_buff[8] << 8) | (int16_t)calib_buff[7];
		_calib_data.par_p2 = ((float)raw_par_p2 - 16384) / temp_var;
		// PAR_P3
		temp_var = 4294967296.f;
		raw_par_p3 = (int8_t)calib_buff[9];
		_calib_data.par_p3 = (float)raw_par_p3 / temp_var;
		// PAR_P4
		temp_var = 137438953472.f;
		raw_par_p4 = (int8_t)calib_buff[10];
		_calib_data.par_p4 = (float)raw_par_p4 / temp_var;
		// PAR_P5
		temp_var = 0.125f;
		raw_par_p5 = ((uint16_t)calib_buff[12] << 8) | (uint16_t)calib_buff[11];
		_calib_data.par_p5 = (float)raw_par_p5 / temp_var;
		// PAR_P6
		temp_var = 64.f;
		raw_par_p6 = ((uint16_t)calib_buff[14] << 8) | (uint16_t)calib_buff[13];
		_calib_data.par_p6 = (float)raw_par_p6 / temp_var;
		// PAR_P7
		temp_var = 256.f;
		raw_par_p7 = (int8_t)calib_buff[15];
		_calib_data.par_p7 = (float)raw_par_p7 / temp_var;
		// PAR_P8
		temp_var = 32768.f;
		raw_par_p8 = (int8_t)calib_buff[16];
		_calib_data.par_p8 = (float)raw_par_p8 / temp_var;
		// PAR_P9
		temp_var = 281474976710656.f;
		raw_par_p9 = ((int16_t)calib_buff[18] << 8) | (int16_t)calib_buff[17];
		_calib_data.par_p9 = (float)raw_par_p9 / temp_var;
		// PAR_P10
		temp_var = 281474976710656.f;
		raw_par_p10 = (int8_t)calib_buff[19];
		_calib_data.par_p10 = (float)raw_par_p10 / temp_var;
		// PAR_P11
		temp_var = 36893488147419103232.f;
		raw_par_p11 = (int8_t)calib_buff[20];
		_calib_data.par_p11 = (float)raw_par_p11 / temp_var;
	}

	return rslt;
}




/*!
 *  @brief Function to compensate raw temperature data
 *
 *  @param none
 *
 *  @return Status of execution
 *  @retval = HAL_OK  		-> Success
 *  @retval != HAL_OK	  	-> Failure Info
 *
 */
float BMP388::compensateTemp(){
	uint32_t uncomp_temp = _raw_temp;

    float partial_data1;
    float partial_data2;

    partial_data1 = (float)(uncomp_temp - _calib_data.par_t1);
    partial_data2 = (float)(partial_data1 * _calib_data.par_t2);

    _temp = partial_data2 + (partial_data1 * partial_data1) * _calib_data.par_t3;

    return _temp;
}




/*!
 *  @brief Function to compensate raw pressure data
 *
 *  @param none
 *
 *  @return Status of execution
 *  @retval = press		: Compensated pressure value
 *
 */
float BMP388::compensatePressure(){
    // Temporary variables used for compensation
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;


    partial_data1 = _calib_data.par_p6 * _temp;
    partial_data2 = _calib_data.par_p7 * (_temp * _temp);
    partial_data3 = _calib_data.par_p8 * (_temp * _temp * _temp);
    partial_out1 = _calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = _calib_data.par_p2 * _temp;
    partial_data2 = _calib_data.par_p3 * (_temp * _temp);
    partial_data3 = _calib_data.par_p4 * (_temp * _temp * _temp);
    partial_out2 = (float)_raw_press * (_calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)_raw_press * (float)_raw_press;
    partial_data2 = _calib_data.par_p9 + _calib_data.par_p10 * _temp;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)_raw_press * (float)_raw_press * (float)_raw_press) * _calib_data.par_p11;

    _press = partial_out1 + partial_out2 + partial_data4;

    return _press;
}




/*!
 *  @brief Function to read byte from BMP388
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] buff	    : Pointer to the data buffer to store the read data.
 *  @param[in] len          : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval = HAL_OK 		-> Success
 *  @retval != HAL_ERROR 	-> Failure Info
 *
 */
HAL_StatusTypeDef BMP388::readBytes(BMP388_regs reg_addr, uint8_t *buff, uint8_t len){
	return HAL_I2C_Mem_Read(_hi2c_ptr, BMP388_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, buff, len, 100);
}




/*!
 *  @brief Function to write byte to BMP388
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] buff	  	    : Pointer to the data buffer to store the read data.
 *  @param[in] len          : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval = HAL_OK 		-> Success
 *  @retval != HAL_OK	 	-> Failure Info
 *
 */
HAL_StatusTypeDef BMP388::writeBytes(BMP388_regs reg_addr, uint8_t *buff, uint8_t len){
	return HAL_I2C_Mem_Write(_hi2c_ptr, BMP388_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, buff, len, 100);
}
