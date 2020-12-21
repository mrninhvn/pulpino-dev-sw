/*
    sensor.cpp
    Driver for ADENT4IOT sensors

    Copyright (c) 2020 SISLAB, VNU.
    Website    : http://sis.uet.vnu.edu.vn/
    Author     : Ninh.D.H
    Create Time: 13 Aug 2020
    Version    : 1.0
    Change Log :
*/

#include "sensor.h"

/*
 * ***********************************************************
   SHT35
 * ***********************************************************
*/
ClosedCube_SHT31D::ClosedCube_SHT31D()
{
}

SHT31D_ErrorCode ClosedCube_SHT31D::begin(uint8_t address) {
	SHT31D_ErrorCode error = SHT3XD_NO_ERROR;
	_address = address;
	return error;
}

SHT31D_ErrorCode ClosedCube_SHT31D::reset()
{
	return  softReset();
}

SHT31D ClosedCube_SHT31D::periodicFetchData()
{
	SHT31D_ErrorCode error = writeCommand(SHT3XD_CMD_FETCH_DATA);
	if (error == SHT3XD_NO_ERROR)
		return readTemperatureAndHumidity();
	else
		returnError(error);
}

SHT31D_ErrorCode ClosedCube_SHT31D::periodicStop() {
	return writeCommand(SHT3XD_CMD_STOP_PERIODIC);
}

SHT31D_ErrorCode ClosedCube_SHT31D::periodicStart(SHT31D_Repeatability repeatability, SHT31D_Frequency frequency)
{
	SHT31D_ErrorCode error;

	switch (repeatability)
	{
	case SHT3XD_REPEATABILITY_LOW:
		switch (frequency)
		{
		case SHT3XD_FREQUENCY_HZ5:
			error = writeCommand(SHT3XD_CMD_PERIODIC_HALF_L);
			break;
		case SHT3XD_FREQUENCY_1HZ:
			error = writeCommand(SHT3XD_CMD_PERIODIC_1_L);
			break;
		case SHT3XD_FREQUENCY_2HZ:
			error = writeCommand(SHT3XD_CMD_PERIODIC_2_L);
			break;
		case SHT3XD_FREQUENCY_4HZ:
			error = writeCommand(SHT3XD_CMD_PERIODIC_4_L);
			break;
		case SHT3XD_FREQUENCY_10HZ:
			error = writeCommand(SHT3XD_CMD_PERIODIC_10_L);
			break;
		default:
			error = SHT3XD_PARAM_WRONG_FREQUENCY;
			break;
		}
		break;
	case SHT3XD_REPEATABILITY_MEDIUM:
		switch (frequency)
		{
		case SHT3XD_FREQUENCY_HZ5:
			error = writeCommand(SHT3XD_CMD_PERIODIC_HALF_M);
			break;
		case SHT3XD_FREQUENCY_1HZ:
			error = writeCommand(SHT3XD_CMD_PERIODIC_1_M);
			break;
		case SHT3XD_FREQUENCY_2HZ:
			error = writeCommand(SHT3XD_CMD_PERIODIC_2_M);
			break;
		case SHT3XD_FREQUENCY_4HZ:
			error = writeCommand(SHT3XD_CMD_PERIODIC_4_M);
			break;
		case SHT3XD_FREQUENCY_10HZ:
			error = writeCommand(SHT3XD_CMD_PERIODIC_10_M);
			break;
		default:
			error = SHT3XD_PARAM_WRONG_FREQUENCY;
			break;
		}
		break;

	case SHT3XD_REPEATABILITY_HIGH:
		switch (frequency)
		{
		case SHT3XD_FREQUENCY_HZ5:
			error = writeCommand(SHT3XD_CMD_PERIODIC_HALF_H);
			break;
		case SHT3XD_FREQUENCY_1HZ:
			error = writeCommand(SHT3XD_CMD_PERIODIC_1_H);
			break;
		case SHT3XD_FREQUENCY_2HZ:
			error = writeCommand(SHT3XD_CMD_PERIODIC_2_H);
			break;
		case SHT3XD_FREQUENCY_4HZ:
			error = writeCommand(SHT3XD_CMD_PERIODIC_4_H);
			break;
		case SHT3XD_FREQUENCY_10HZ:
			error = writeCommand(SHT3XD_CMD_PERIODIC_10_H);
			break;
		default:
			error = SHT3XD_PARAM_WRONG_FREQUENCY;
			break;
		}
		break;
	default:
		error = SHT3XD_PARAM_WRONG_REPEATABILITY;
		break;
	}

	delay(100);

	return error;
}

SHT31D ClosedCube_SHT31D::readTempAndHumidity(SHT31D_Repeatability repeatability, SHT31D_Mode mode, uint8_t timeout)
{
	SHT31D result;

	switch (mode) {
	case SHT3XD_MODE_CLOCK_STRETCH:
		result = readTempAndHumidityClockStretch(repeatability);
		break;
	case SHT3XD_MODE_POLLING:
		result = readTempAndHumidityPolling(repeatability, timeout);
		break;
	default:
		result = returnError(SHT3XD_PARAM_WRONG_MODE);
		break;
	}

	return result;
}


SHT31D ClosedCube_SHT31D::readTempAndHumidityClockStretch(SHT31D_Repeatability repeatability)
{
	SHT31D_ErrorCode error = SHT3XD_NO_ERROR;
	SHT31D_Commands command;

	switch (repeatability)
	{
	case SHT3XD_REPEATABILITY_LOW:
		error = writeCommand(SHT3XD_CMD_CLOCK_STRETCH_L);
		break;
	case SHT3XD_REPEATABILITY_MEDIUM:
		error = writeCommand(SHT3XD_CMD_CLOCK_STRETCH_M);
		break;
	case SHT3XD_REPEATABILITY_HIGH:
		error = writeCommand(SHT3XD_CMD_CLOCK_STRETCH_H);
		break;
	default:
		error = SHT3XD_PARAM_WRONG_REPEATABILITY;
		break;
	}

	delay(50);

	if (error == SHT3XD_NO_ERROR) {
		return readTemperatureAndHumidity();
	} else {
		return returnError(error);
	}

}


SHT31D ClosedCube_SHT31D::readTempAndHumidityPolling(SHT31D_Repeatability repeatability, uint8_t timeout)
{
	SHT31D_ErrorCode error = SHT3XD_NO_ERROR;
	SHT31D_Commands command;

	switch (repeatability)
	{
	case SHT3XD_REPEATABILITY_LOW:
		error = writeCommand(SHT3XD_CMD_POLLING_L);
		break;
	case SHT3XD_REPEATABILITY_MEDIUM:
		error = writeCommand(SHT3XD_CMD_POLLING_M);
		break;
	case SHT3XD_REPEATABILITY_HIGH:
		error = writeCommand(SHT3XD_CMD_POLLING_H);
		break;
	default:
		error = SHT3XD_PARAM_WRONG_REPEATABILITY;
		break;
	}

	delay(50);

	if (error == SHT3XD_NO_ERROR) {
		return readTemperatureAndHumidity();
	} else {
		return returnError(error);
	}

}

SHT31D ClosedCube_SHT31D::readAlertHighSet() {
	return readAlertData(SHT3XD_CMD_READ_ALR_LIMIT_HS);
}

SHT31D ClosedCube_SHT31D::readAlertHighClear() {
	return readAlertData(SHT3XD_CMD_READ_ALR_LIMIT_HC);
}

SHT31D ClosedCube_SHT31D::readAlertLowSet() {
	return readAlertData(SHT3XD_CMD_READ_ALR_LIMIT_LS);
}

SHT31D ClosedCube_SHT31D::readAlertLowClear() {
	return readAlertData(SHT3XD_CMD_READ_ALR_LIMIT_LC);
}


SHT31D_ErrorCode ClosedCube_SHT31D::writeAlertHigh(float temperatureSet, float temperatureClear, float humiditySet, float humidityClear) {
	SHT31D_ErrorCode error = writeAlertData(SHT3XD_CMD_WRITE_ALR_LIMIT_HS, temperatureSet, humiditySet);
	if (error == SHT3XD_NO_ERROR)
		error = writeAlertData(SHT3XD_CMD_WRITE_ALR_LIMIT_HC, temperatureClear, humidityClear);

	return error;
}

SHT31D_ErrorCode ClosedCube_SHT31D::writeAlertLow(float temperatureClear, float temperatureSet, float humidityClear, float humiditySet) {
	SHT31D_ErrorCode error = writeAlertData(SHT3XD_CMD_WRITE_ALR_LIMIT_LS, temperatureSet, humiditySet);
	if (error == SHT3XD_NO_ERROR)
		writeAlertData(SHT3XD_CMD_WRITE_ALR_LIMIT_LC, temperatureClear, humidityClear);

	return error;
}

SHT31D_ErrorCode ClosedCube_SHT31D::writeAlertData(SHT31D_Commands command, float temperature, float humidity)
{
	SHT31D_ErrorCode  error;

	if ((humidity < 0.0) || (humidity > 100.0) || (temperature < -40.0) || (temperature > 125.0))
	{
		error = SHT3XD_PARAM_WRONG_ALERT;
	} else
	{
		uint16_t rawTemperature = calculateRawTemperature(temperature);
		uint16_t rawHumidity = calculateRawHumidity(humidity);
		uint16_t data = (rawHumidity & 0xFE00) | ((rawTemperature >> 7) & 0x001FF);

		uint8_t	buf[2];
		buf[0] = data >> 8;
		buf[1] = data & 0xFF;

		uint8_t checksum = calculateCrc(buf);

		Wire.beginTransmission(_address);
		Wire.write(command >> 8);
		Wire.write(command & 0xFF);
		Wire.write(buf[0]);
		Wire.write(buf[1]);
		Wire.write(checksum);
		return (SHT31D_ErrorCode)(-10 * Wire.endTransmission());
	}

	return error;
}


SHT31D_ErrorCode ClosedCube_SHT31D::writeCommand(SHT31D_Commands command)
{
	Wire.beginTransmission(_address);
	Wire.write(command >> 8);
	Wire.write(command & 0xFF);
	return (SHT31D_ErrorCode)(-10 * Wire.endTransmission());
}

SHT31D_ErrorCode ClosedCube_SHT31D::softReset() {
	return writeCommand(SHT3XD_CMD_SOFT_RESET);
}

SHT31D_ErrorCode ClosedCube_SHT31D::generalCallReset() {
	Wire.beginTransmission(0x0);
	Wire.write(0x06);
	return (SHT31D_ErrorCode)(-10 * Wire.endTransmission());
}

SHT31D_ErrorCode ClosedCube_SHT31D::heaterEnable() {
	return writeCommand(SHT3XD_CMD_HEATER_ENABLE);
}

SHT31D_ErrorCode ClosedCube_SHT31D::heaterDisable() {
	return writeCommand(SHT3XD_CMD_HEATER_DISABLE);
}

SHT31D_ErrorCode ClosedCube_SHT31D::artEnable() {
	return writeCommand(SHT3XD_CMD_ART);
}


uint32_t ClosedCube_SHT31D::readSerialNumber()
{
	uint32_t result = SHT3XD_NO_ERROR;
	uint16_t buf[2];

	if (writeCommand(SHT3XD_CMD_READ_SERIAL_NUMBER) == SHT3XD_NO_ERROR) {
		if (read(buf, 2) == SHT3XD_NO_ERROR) {
			result = (buf[0] << 16) | buf[1];
		}
	}

	return result;
}

SHT31D_RegisterStatus ClosedCube_SHT31D::readStatusRegister()
{
	SHT31D_RegisterStatus result;

	SHT31D_ErrorCode error = writeCommand(SHT3XD_CMD_READ_STATUS);
	if (error == SHT3XD_NO_ERROR)
		error = read(&result.rawData, 1);

	return result;
}

SHT31D_ErrorCode ClosedCube_SHT31D::clearAll() {
	return writeCommand(SHT3XD_CMD_CLEAR_STATUS);
}


SHT31D ClosedCube_SHT31D::readTemperatureAndHumidity()
{
	SHT31D result;
	result.t = 0;
	result.rh = 0;

	SHT31D_ErrorCode error;
	uint16_t buf[2];

	if (error == SHT3XD_NO_ERROR)
		error = read(buf, 2);

	if (error == SHT3XD_NO_ERROR) {
		result.t = calculateTemperature(buf[0]);
		result.rh = calculateHumidity(buf[1]);
	}
	result.error = error;

	return result;
}

SHT31D ClosedCube_SHT31D::readAlertData(SHT31D_Commands command)
{
	SHT31D result;

	result.t = 0;
	result.rh = 0;

	SHT31D_ErrorCode error;
	
	uint16_t buf;

	error = writeCommand(command);

	if (error == SHT3XD_NO_ERROR)
		error = read(&buf, 1);

	if (error == SHT3XD_NO_ERROR) {
		result.rh = calculateHumidity(buf & 0xFE00);
		result.t = calculateTemperature(buf << 7);
	}

	result.error = error;

	return result;
}

SHT31D_ErrorCode ClosedCube_SHT31D::read(uint16_t* data, uint8_t numOfPair)
{
	uint8_t	buf[2];
	uint8_t checksum;

	const uint8_t numOfBytes = numOfPair * 3;
	Wire.requestFrom(_address, numOfBytes);

	int counter = 0;

	for (counter = 0; counter < numOfPair; counter++) {
		Wire.readBytes(buf, (uint8_t)2);
		checksum = Wire.read();

		if (checkCrc(buf, checksum) != 0)
			return SHT3XD_CRC_ERROR;

		data[counter] = (buf[0] << 8) | buf[1];
	}

	return SHT3XD_NO_ERROR;
}


uint8_t ClosedCube_SHT31D::checkCrc(uint8_t data[], uint8_t checksum)
{
	return calculateCrc(data) != checksum;
}

float ClosedCube_SHT31D::calculateTemperature(uint16_t rawValue)
{
	return 175.0f * (float)rawValue / 65535.0f - 45.0f;
}


float ClosedCube_SHT31D::calculateHumidity(uint16_t rawValue)
{
	return 100.0f * rawValue / 65535.0f;
}

uint16_t ClosedCube_SHT31D::calculateRawTemperature(float value)
{
	return (value + 45.0f) / 175.0f * 65535.0f;
}

uint16_t ClosedCube_SHT31D::calculateRawHumidity(float value)
{
	return value / 100.0f * 65535.0f;
}

uint8_t ClosedCube_SHT31D::calculateCrc(uint8_t data[])
{
	uint8_t bit;
	uint8_t crc = 0xFF;
	uint8_t dataCounter = 0;

	for (; dataCounter < 2; dataCounter++)
	{
		crc ^= (data[dataCounter]);
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80)
				crc = (crc << 1) ^ 0x131;
			else
				crc = (crc << 1);
		}
	}

	return crc;
}

SHT31D ClosedCube_SHT31D::returnError(SHT31D_ErrorCode error) {
	SHT31D result;
	result.t = 0;
	result.rh = 0;
	result.error = error;
	return result;
}

/*
 * ***********************************************************
   HM330X
 * ***********************************************************
*/

/**
    @brief I2C write byte
    @param reg :Register address of operation object
    @param byte :The byte to be wrote.
    @return result of operation,non-zero if failed.
*/
ErrorCode HM330X_I2C::IIC_write_byte(uint8_t reg, uint8_t byte) {
    int ret = 0;
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(reg);
    Wire.write(byte);
    ret = Wire.endTransmission();
    if (!ret) {
        return NO_ERROR;
    } else {
        return ERROR_COMM;
    }
}

/**
    @brief I2C write 16bit value
    @param reg: Register address of operation object
    @param value: The 16bit value to be wrote .
    @return result of operation,non-zero if failed.
*/
ErrorCode HM330X_I2C::IIC_write_16bit(uint8_t reg, uint16_t value) {
    int ret = 0;
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(reg);

    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t) value);
    ret = Wire.endTransmission();
    if (!ret) {
        return NO_ERROR;
    } else {
        return ERROR_COMM;
    }
}

/**
    @brief I2C read byte
    @param reg: Register address of operation object
    @param byte: The byte to be read in.
    @return result of operation,non-zero if failed.
*/
ErrorCode HM330X_I2C::IIC_read_byte(uint8_t reg, uint8_t* byte) {
    uint32_t time_out_count = 0;
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_IIC_ADDR, (uint8_t) 1);
    while (1 != Wire.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }
    *byte = Wire.read();
    return NO_ERROR;
}

/**
    @brief I2C read 16bit value
    @param reg: Register address of operation object
    @param byte: The 16bit value to be read in.
    @return result of operation,non-zero if failed.
*/
ErrorCode HM330X_I2C::IIC_read_16bit(uint8_t start_reg, uint16_t* value) {
    uint32_t time_out_count = 0;
    uint8_t val = 0;
    *value = 0;
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(start_reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_IIC_ADDR, sizeof(uint16_t));
    while (sizeof(uint16_t) != Wire.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }
    val = Wire.read();
    *value |= (uint16_t) val << 8;
    val = Wire.read();
    *value |= val;
    return NO_ERROR;
}

/**
    @brief I2C read some bytes
    @param reg: Register address of operation object
    @param data: The buf  to be read in.
    @param data_len: The length of buf need to read in.
    @return result of operation,non-zero if failed.
*/
ErrorCode HM330X_I2C::IIC_read_bytes(uint8_t start_reg, uint8_t* data, uint32_t data_len) {
    ErrorCode ret = NO_ERROR;
    uint32_t time_out_count = 0;
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(start_reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_IIC_ADDR, data_len);
    while (data_len != Wire.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            return ERROR_COMM;
        }
        delay(1);
    }

    for (int i = 0; i < data_len; i++) {
        data[i] = Wire.read();
    }
    return ret;
}

/**
    @brief change the I2C address from default.
    @param IIC_ADDR: I2C address to be set
*/
void HM330X_I2C::set_iic_addr(uint8_t IIC_ADDR) {
    _IIC_ADDR = IIC_ADDR;
}

ErrorCode HM330X_I2C::IIC_SEND_CMD(uint8_t CMD) {
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(CMD);
    byte ret = Wire.endTransmission();
    if (ret == 0) {
        return NO_ERROR;
    } else {
        return ERROR_COMM;
    }
}


HM330X::HM330X(uint8_t IIC_ADDR) {
    set_iic_addr(IIC_ADDR);
}

ErrorCode HM330X::select_comm() {
    return IIC_SEND_CMD(SELECT_COMM_CMD);
}

ErrorCode HM330X::init() {
    Wire.begin();
    return select_comm();
}

ErrorCode HM330X::read_sensor_value(uint8_t* data, uint32_t data_len) {
    if (i2c_busy)
    {
      Wire.clear_busy();
    }
    uint32_t time_out_count = 0;
    ErrorCode ret = NO_ERROR;
    Wire.requestFrom(0x40, 29);
    while (data_len != Wire.available()) {
        time_out_count++;
        if (time_out_count > 10) {
            //Wire.clear_busy();
            return ERROR_COMM;
        }
        delay(1);
    }
    for (int i = 0; i < data_len; i++) {
        data[i] = Wire.read();
    }
    if (i2c_busy)
    {
      Wire.clear_busy();
    }
    return ret;
}

ErrorCode HM330X::parse_result(uint8_t* dataIn, uint16_t* dataOut) {
    if (NULL == dataIn) {
        return ERROR_PARAM;
    }
    uint8_t sum = 0;
    for (int i = 0; i < 28; i++) {
        sum += dataIn[i];
    }
    if (sum != dataIn[28]) {
        //Serial.println("wrong checkSum!!!!");
        return ERROR_SUM;
    }
    for (int i = 1; i < 8; i++) {
        dataOut[i-1] = (uint16_t) dataIn[i * 2] << 8 | dataIn[i * 2 + 1];

    }

    return NO_ERROR;
}

/*
 * ***********************************************************
   SGP30
 * ***********************************************************
*/

//Constructor
SGP30::SGP30()
{
  CO2 = 0;
  TVOC = 0;
  baselineCO2 = 0;
  baselineTVOC = 0;
  featureSetVersion = 0;
  H2 = 0;
  ethanol = 0;
  serialID = 0;
}

//Start I2C communication using specified port
//Returns true if successful or false if no sensor detected
bool SGP30::begin(TwoWire &wirePort)
{
  _i2cPort = &wirePort; //Grab which port the user wants us to use
  _i2cPort->begin();
  getSerialID();
  if (serialID == 0)
    return false;
  return true;
}

//Initilizes sensor for air quality readings
//measureAirQuality should be called in 1 second intervals after this function
void SGP30::initAirQuality(void)
{
  _i2cPort->beginTransmission(_SGP30Address);
  _i2cPort->write(init_air_quality, 2); //command to initialize air quality readings
  _i2cPort->endTransmission();
}

//Measure air quality
//Call in regular intervals of 1 second to maintain synamic baseline calculations
//CO2 returned in ppm, Total Volatile Organic Compounds (TVOC) returned in ppb
//Will give fixed values of CO2=400 and TVOC=0 for first 15 seconds after init
//Returns SUCCESS if successful or other error code if unsuccessful
SGP30ERR SGP30::measureAirQuality(void)
{
  _i2cPort->beginTransmission(_SGP30Address);
  _i2cPort->write(measure_air_quality, 2); //command to measure air quality
  _i2cPort->endTransmission();
  //Hang out while measurement is taken. datasheet says 10-12ms
  delay(12);
  //Comes back in 6 bytes, CO2 data(MSB) / data(LSB) / Checksum / TVOC data(MSB) / data(LSB) / Checksum
  uint8_t toRead;
  toRead = _i2cPort->requestFrom(_SGP30Address, (uint8_t)6);
  if (toRead != 6)
  {
    _i2cPort->clear_rxa();
    return ERR_I2C_TIMEOUT;              //Error out
  }
  uint16_t _CO2 = _i2cPort->read() << 8; //store MSB in CO2
  _CO2 |= _i2cPort->read();              //store LSB in CO2
  uint8_t checkSum = _i2cPort->read();   //verify checksum
  if (checkSum != _CRC8(_CO2))
    return ERR_BAD_CRC;                   //checksum failed
  uint16_t _TVOC = _i2cPort->read() << 8; //store MSB in TVOC
  _TVOC |= _i2cPort->read();              //store LSB in TVOC
  checkSum = _i2cPort->read();            //verify checksum
  if (checkSum != _CRC8(_TVOC))
    return ERR_BAD_CRC; //checksum failed
  CO2 = _CO2;           //publish valid data
  TVOC = _TVOC;         //publish valid data
  return SUCCESS;
}

//Returns the current calculated baseline from
//the sensor's dynamic baseline calculations
//Save baseline periodically to non volatile memory
//(like EEPROM) to restore after new power up or
//after soft reset using setBaseline();
//Returns SUCCESS if successful or other error code if unsuccessful
SGP30ERR SGP30::getBaseline(void)
{
  _i2cPort->beginTransmission(_SGP30Address);
  _i2cPort->write(get_baseline, 2);
  _i2cPort->endTransmission();
  //Hang out while measurement is taken. datasheet says 10ms
  delay(10);
  uint8_t toRead;
  //Comes back in 6 bytes, baselineCO2 data(MSB) / data(LSB) / Checksum / baselineTVOC data(MSB) / data(LSB) / Checksum
  toRead = _i2cPort->requestFrom(_SGP30Address, (uint8_t)6);
  if (toRead != 6)
    return ERR_I2C_TIMEOUT;                      //Error out
  uint16_t _baselineCO2 = _i2cPort->read() << 8; //store MSB in _baselineCO2
  _baselineCO2 |= _i2cPort->read();              //store LSB in _baselineCO2
  uint8_t checkSum = _i2cPort->read();           //verify checksum
  if (checkSum != _CRC8(_baselineCO2))
    return ERR_BAD_CRC;                           //checksum failed
  uint16_t _baselineTVOC = _i2cPort->read() << 8; //store MSB in _baselineTVOC
  _baselineTVOC |= _i2cPort->read();              //store LSB in _baselineTVOC
  checkSum = _i2cPort->read();                    //verify checksum
  if (checkSum != _CRC8(_baselineTVOC))
    return ERR_BAD_CRC;         //checksum failed
  baselineCO2 = _baselineCO2;   //publish valid data
  baselineTVOC = _baselineTVOC; //publish valid data
  return SUCCESS;
}

//Updates the baseline to a previous baseline
//Should only use with previously retrieved baselines
//to maintain accuracy
void SGP30::setBaseline(uint16_t baselineCO2, uint16_t baselineTVOC)
{
  _i2cPort->beginTransmission(_SGP30Address);
  _i2cPort->write(set_baseline, 2);     //command to set baseline
  _i2cPort->write(baselineTVOC >> 8);   //write baseline TVOC MSB
  _i2cPort->write(baselineTVOC);        //write baseline TVOC LSB
  _i2cPort->write(_CRC8(baselineTVOC)); //write checksum TVOC baseline
  _i2cPort->write(baselineCO2 >> 8);    //write baseline CO2 MSB
  _i2cPort->write(baselineCO2);         //write baseline CO2 LSB
  _i2cPort->write(_CRC8(baselineCO2));  //write checksum CO2 baseline
  _i2cPort->endTransmission();
}

//Set humidity
//humidity value is a fixed point 8.8 bit number
//Value should be absolute humidity from humidity sensor
//default value 0x0F80 = 15.5g/m^3
//minimum value 0x0001 = 1/256g/m^3
//maximum value 0xFFFF = 255+255/256 g/m^3
//sending 0x0000 resets to default and turns off humidity compensation
void SGP30::setHumidity(uint16_t humidity)
{
  _i2cPort->beginTransmission(_SGP30Address);
  _i2cPort->write(set_humidity, 2); //command to set humidity
  _i2cPort->write(humidity >> 8);   //write humidity MSB
  _i2cPort->write(humidity);        //write humidity LSB
  _i2cPort->write(_CRC8(humidity)); //write humidity checksum
  _i2cPort->endTransmission();
}

//gives feature set version number (see data sheet)
//Returns SUCCESS if successful or other error code if unsuccessful
SGP30ERR SGP30::getFeatureSetVersion(void)
{
  _i2cPort->beginTransmission(_SGP30Address);
  _i2cPort->write(get_feature_set_version, 2); //command to get feature version
  _i2cPort->endTransmission();
  //Hang out while measurement is taken. datasheet says 1-2ms
  delay(2);
  uint8_t toRead;
  //Comes back in 3 bytes, data(MSB) / data(LSB) / Checksum
  toRead = _i2cPort->requestFrom(_SGP30Address, (uint8_t)3);
  if (toRead != 3)
    return ERR_I2C_TIMEOUT;                            //Error out
  uint16_t _featureSetVersion = _i2cPort->read() << 8; //store MSB in featureSetVerison
  _featureSetVersion |= _i2cPort->read();              //store LSB in featureSetVersion
  uint8_t checkSum = _i2cPort->read();                 //verify checksum
  if (checkSum != _CRC8(_featureSetVersion))
    return ERR_BAD_CRC;                   //checksum failed
  featureSetVersion = _featureSetVersion; //publish valid data
  return SUCCESS;
}

//Intended for part verification and testing
//these raw signals are used as inputs to the onchip calibrations and algorithms
//Returns SUCCESS if successful or other error code if unsuccessful
SGP30ERR SGP30::measureRawSignals(void)
{
  _i2cPort->beginTransmission(_SGP30Address);
  _i2cPort->write(measure_raw_signals, 2); //command to measure raw signals
  _i2cPort->endTransmission();
  //Hang out while measurement is taken. datasheet says 20-25ms
  delay(25);
  uint8_t toRead;
  //Comes back in 6 bytes, H2 data(MSB) / data(LSB) / Checksum / ethanol data(MSB) / data(LSB) / Checksum
  toRead = _i2cPort->requestFrom(_SGP30Address, (uint8_t)6);
  if (toRead != 6)
    return ERR_I2C_TIMEOUT;             //Error out
  uint16_t _H2 = _i2cPort->read() << 8; //store MSB in _H2
  _H2 |= _i2cPort->read();              //store LSB in _H2
  uint8_t checkSum = _i2cPort->read();  //verify checksum
  if (checkSum != _CRC8(_H2))
    return ERR_BAD_CRC;                      //checksumfailed
  uint16_t _ethanol = _i2cPort->read() << 8; //store MSB in ethanol
  _ethanol |= _i2cPort->read();              //store LSB in ethanol
  checkSum = _i2cPort->read();               //verify checksum
  if (checkSum != _CRC8(_ethanol))
    return ERR_BAD_CRC; //checksum failed
  H2 = _H2;             //publish valid data
  ethanol = _ethanol;   //publish valid data
  return SUCCESS;
}

//Soft reset - not device specific
//will reset all devices that support general call mode
void SGP30::generalCallReset(void)
{
  _i2cPort->beginTransmission(0x00); //general call address
  _i2cPort->write(0x06);             //reset command
  _i2cPort->endTransmission();
}

//readout of serial ID register can identify chip and verify sensor presence
//Returns SUCCESS if successful or other error code if unsuccessful
SGP30ERR SGP30::getSerialID(void)
{
  _i2cPort->beginTransmission(_SGP30Address);
  _i2cPort->write(get_serial_id, 2); //command to get serial ID
  _i2cPort->endTransmission();
  //Hang out while measurement is taken.
  delay(1);
  uint8_t toRead;
  //Comes back in 9 bytes, H2 data(MSB) / data(LSB) / Checksum / ethanol data(MSB) / data(LSB) / Checksum
  toRead = _i2cPort->requestFrom(_SGP30Address, (uint8_t)9);
  if (toRead != 9)
    return ERR_I2C_TIMEOUT;                    //Error out
  uint16_t _serialID1 = _i2cPort->read() << 8; //store MSB to top of _serialID1
  _serialID1 |= _i2cPort->read();              //store next byte in _serialID1
  uint8_t checkSum1 = _i2cPort->read();        //verify checksum
  if (checkSum1 != _CRC8(_serialID1))
    return ERR_BAD_CRC;                        //checksum failed
  uint16_t _serialID2 = _i2cPort->read() << 8; //store next byte to top of _serialID2
  _serialID2 |= _i2cPort->read();              //store next byte in _serialID2
  uint8_t checkSum2 = _i2cPort->read();        //verify checksum
  if (checkSum2 != _CRC8(_serialID2))
    return ERR_BAD_CRC;                        //checksum failed
  uint16_t _serialID3 = _i2cPort->read() << 8; //store next byte to top of _serialID3
  _serialID3 |= _i2cPort->read();              //store LSB in _serialID3
  uint8_t checkSum3 = _i2cPort->read();        //verify checksum
  if (checkSum3 != _CRC8(_serialID3))
    return ERR_BAD_CRC;                                                                            //checksum failed
  serialID = ((uint64_t)_serialID1 << 32) + ((uint64_t)_serialID2 << 16) + ((uint64_t)_serialID3); //publish valid data
  return SUCCESS;
}

//Sensor runs on chip self test
//Returns SUCCESS if successful or other error code if unsuccessful
SGP30ERR SGP30::measureTest(void)
{
  _i2cPort->beginTransmission(_SGP30Address);
  _i2cPort->write(measure_test, 2); //command to get self test
  _i2cPort->endTransmission();
  //Hang out while measurement is taken. datasheet says 200-220ms
  delay(220);
  uint8_t toRead;
  //Comes back in 3 bytes, data(MSB) / data(LSB) / Checksum
  toRead = _i2cPort->requestFrom(_SGP30Address, (uint8_t)3);
  if (toRead != 3)
    return ERR_I2C_TIMEOUT;                 //Error out
  uint16_t results = _i2cPort->read() << 8; //store MSB in results
  results |= _i2cPort->read();              //store LSB in results
  uint8_t checkSum = _i2cPort->read();      //verify checksum
  if (checkSum != _CRC8(results))
    return ERR_BAD_CRC; //checksum failed
  if (results != 0xD400)
    return SELF_TEST_FAIL; //self test results incorrect
  return SUCCESS;
}

#ifndef SGP30_LOOKUP_TABLE
//Given an array and a number of bytes, this calculate CRC8 for those bytes
//CRC is only calc'd on the data portion (two bytes) of the four bytes being sent
//From: http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
//Tested with: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
//x^8+x^5+x^4+1 = 0x31
uint8_t SGP30::_CRC8(uint16_t data)
{
  uint8_t crc = 0xFF; //Init with 0xFF

  crc ^= (data >> 8); // XOR-in the first input byte

  for (uint8_t i = 0; i < 8; i++)
  {
    if ((crc & 0x80) != 0)
      crc = (uint8_t)((crc << 1) ^ 0x31);
    else
      crc <<= 1;
  }
  crc ^= (uint8_t)data; // XOR-in the last input byte

  for (uint8_t i = 0; i < 8; i++)
  {
    if ((crc & 0x80) != 0)
      crc = (uint8_t)((crc << 1) ^ 0x31);
    else
      crc <<= 1;
  }

  return crc; //No output reflection
}
#endif

#ifdef SGP30_LOOKUP_TABLE
//Generates CRC8 for SGP30 from lookup table
uint8_t SGP30::_CRC8(uint16_t data)
{
  uint8_t CRC = 0xFF;                          //inital value
  CRC ^= (uint8_t)(data >> 8);                 //start with MSB
  CRC = _CRC8LookupTable[CRC >> 4][CRC & 0xF]; //look up table [MSnibble][LSnibble]
  CRC ^= (uint8_t)data;                        //use LSB
  CRC = _CRC8LookupTable[CRC >> 4][CRC & 0xF]; //look up table [MSnibble][LSnibble]
  return CRC;
}
#endif
