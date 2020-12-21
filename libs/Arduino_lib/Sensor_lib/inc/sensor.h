/*
    sensor.h
    Driver for ADENT4IOT sensors

    Copyright (c) 2020 SISLAB, VNU.
    Website    : http://sis.uet.vnu.edu.vn/
    Author     : Ninh.D.H
    Create Time: 13 Aug 2020
    Version    : 1.0
    Change Log :
*/

#ifndef _SENSOR_H
#define _SENSOR_H

#include "Arduino.h"
#include "Wire.h"
#include "OneWire.h"
#include "gpio.h"
extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
  #include "twi.h"
}
#define LTUNUSED(v) (void) (v)

#ifdef SISLAB_PULPINO
#define F(a) a
#define delay(a) delay(20*a)
#define delayMicroseconds(a) delayMicroseconds(20*a)
#define __riscv
#endif

typedef enum {
    NO_ERROR = 0,
    ERROR_PARAM = -1,
    ERROR_COMM = -2,
    ERROR_SUM = -3,
    ERROR_OTHERS = -128,
} ErrorCode;

/*
 * ***********************************************************
   SHT35
 * ***********************************************************
*/

#ifndef CLOSEDCUBE_SHT31D
#define CLOSEDCUBE_SHT31D

typedef enum {
	SHT3XD_CMD_READ_SERIAL_NUMBER = 0x3780,

	SHT3XD_CMD_READ_STATUS = 0xF32D,
	SHT3XD_CMD_CLEAR_STATUS = 0x3041,

	SHT3XD_CMD_HEATER_ENABLE = 0x306D,
	SHT3XD_CMD_HEATER_DISABLE = 0x3066,

	SHT3XD_CMD_SOFT_RESET = 0x30A2,

	SHT3XD_CMD_CLOCK_STRETCH_H = 0x2C06,
	SHT3XD_CMD_CLOCK_STRETCH_M = 0x2C0D,
	SHT3XD_CMD_CLOCK_STRETCH_L = 0x2C10,

	SHT3XD_CMD_POLLING_H = 0x2400,
	SHT3XD_CMD_POLLING_M = 0x240B,
	SHT3XD_CMD_POLLING_L = 0x2416,

	SHT3XD_CMD_ART = 0x2B32,

	SHT3XD_CMD_PERIODIC_HALF_H = 0x2032,
	SHT3XD_CMD_PERIODIC_HALF_M = 0x2024,
	SHT3XD_CMD_PERIODIC_HALF_L = 0x202F,
	SHT3XD_CMD_PERIODIC_1_H = 0x2130,
	SHT3XD_CMD_PERIODIC_1_M = 0x2126,
	SHT3XD_CMD_PERIODIC_1_L = 0x212D,
	SHT3XD_CMD_PERIODIC_2_H = 0x2236,
	SHT3XD_CMD_PERIODIC_2_M = 0x2220,
	SHT3XD_CMD_PERIODIC_2_L = 0x222B,
	SHT3XD_CMD_PERIODIC_4_H = 0x2334,
	SHT3XD_CMD_PERIODIC_4_M = 0x2322,
	SHT3XD_CMD_PERIODIC_4_L = 0x2329,
	SHT3XD_CMD_PERIODIC_10_H = 0x2737,
	SHT3XD_CMD_PERIODIC_10_M = 0x2721,
	SHT3XD_CMD_PERIODIC_10_L = 0x272A,

	SHT3XD_CMD_FETCH_DATA = 0xE000,
	SHT3XD_CMD_STOP_PERIODIC = 0x3093,

	SHT3XD_CMD_READ_ALR_LIMIT_LS = 0xE102,
	SHT3XD_CMD_READ_ALR_LIMIT_LC = 0xE109,
	SHT3XD_CMD_READ_ALR_LIMIT_HS = 0xE11F,
	SHT3XD_CMD_READ_ALR_LIMIT_HC = 0xE114,

	SHT3XD_CMD_WRITE_ALR_LIMIT_HS = 0x611D,
	SHT3XD_CMD_WRITE_ALR_LIMIT_HC = 0x6116,
	SHT3XD_CMD_WRITE_ALR_LIMIT_LC = 0x610B,
	SHT3XD_CMD_WRITE_ALR_LIMIT_LS = 0x6100,

	SHT3XD_CMD_NO_SLEEP = 0x303E,
} SHT31D_Commands;


typedef enum {
	SHT3XD_REPEATABILITY_HIGH,
	SHT3XD_REPEATABILITY_MEDIUM,
	SHT3XD_REPEATABILITY_LOW,
} SHT31D_Repeatability;

typedef enum {
	SHT3XD_MODE_CLOCK_STRETCH,
	SHT3XD_MODE_POLLING,
} SHT31D_Mode;

typedef enum {
	SHT3XD_FREQUENCY_HZ5,
	SHT3XD_FREQUENCY_1HZ,
	SHT3XD_FREQUENCY_2HZ,
	SHT3XD_FREQUENCY_4HZ,
	SHT3XD_FREQUENCY_10HZ
} SHT31D_Frequency;

typedef enum {
	SHT3XD_NO_ERROR = 0,

	SHT3XD_CRC_ERROR = -101,
	SHT3XD_TIMEOUT_ERROR = -102,

	SHT3XD_PARAM_WRONG_MODE = -501,
	SHT3XD_PARAM_WRONG_REPEATABILITY = -502,
	SHT3XD_PARAM_WRONG_FREQUENCY = -503,
	SHT3XD_PARAM_WRONG_ALERT = -504,

	// Wire I2C translated error codes
	SHT3XD_WIRE_I2C_DATA_TOO_LOG = -10,
	SHT3XD_WIRE_I2C_RECEIVED_NACK_ON_ADDRESS = -20,
	SHT3XD_WIRE_I2C_RECEIVED_NACK_ON_DATA = -30,
	SHT3XD_WIRE_I2C_UNKNOW_ERROR = -40
} SHT31D_ErrorCode;

typedef union {
	uint16_t rawData;
	struct {
		uint8_t WriteDataChecksumStatus : 1;
		uint8_t CommandStatus : 1;
		uint8_t Reserved0 : 2;
		uint8_t SystemResetDetected : 1;
		uint8_t Reserved1 : 5;
		uint8_t T_TrackingAlert : 1;
		uint8_t RH_TrackingAlert : 1;
		uint8_t Reserved2 : 1;
		uint8_t HeaterStatus : 1;
		uint8_t Reserved3 : 1;
		uint8_t AlertPending : 1;
	};
} SHT31D_RegisterStatus;

struct SHT31D {
	float t;
	float rh;
	SHT31D_ErrorCode error;
};

class ClosedCube_SHT31D {
public:
	ClosedCube_SHT31D();

	SHT31D_ErrorCode begin(uint8_t address);
	SHT31D_ErrorCode clearAll();
	SHT31D_RegisterStatus readStatusRegister();

	SHT31D_ErrorCode heaterEnable();
	SHT31D_ErrorCode heaterDisable();

	SHT31D_ErrorCode softReset();
	SHT31D_ErrorCode reset(); // same as softReset

	SHT31D_ErrorCode generalCallReset();

	SHT31D_ErrorCode artEnable();

	uint32_t readSerialNumber();

	SHT31D readTempAndHumidity(SHT31D_Repeatability repeatability, SHT31D_Mode mode, uint8_t timeout);
	SHT31D readTempAndHumidityClockStretch(SHT31D_Repeatability repeatability);
	SHT31D readTempAndHumidityPolling(SHT31D_Repeatability repeatability, uint8_t timeout);

	SHT31D_ErrorCode periodicStart(SHT31D_Repeatability repeatability, SHT31D_Frequency frequency);
	SHT31D periodicFetchData();
	SHT31D_ErrorCode periodicStop();

	SHT31D_ErrorCode writeAlertHigh(float temperatureSet, float temperatureClear, float humiditySet, float humidityClear);
	SHT31D readAlertHighSet();
	SHT31D readAlertHighClear();

	SHT31D_ErrorCode writeAlertLow(float temperatureClear, float temperatureSet, float humidityClear, float humiditySet);
	SHT31D readAlertLowSet();
	SHT31D readAlertLowClear();


private:
	uint8_t _address;
	SHT31D_RegisterStatus _status;

	SHT31D_ErrorCode writeCommand(SHT31D_Commands command);
	SHT31D_ErrorCode writeAlertData(SHT31D_Commands command, float temperature, float humidity);

	uint8_t checkCrc(uint8_t data[], uint8_t checksum);
	uint8_t calculateCrc(uint8_t data[]);

	float calculateHumidity(uint16_t rawValue);
	float calculateTemperature(uint16_t rawValue);

	uint16_t calculateRawHumidity(float value);
	uint16_t calculateRawTemperature(float value);

	SHT31D readTemperatureAndHumidity();
	SHT31D readAlertData(SHT31D_Commands command);
	SHT31D_ErrorCode read(uint16_t* data, uint8_t numOfPair);

	SHT31D returnError(SHT31D_ErrorCode command);
};


#endif


/*
 * ***********************************************************
   HM330X
 * ***********************************************************
*/

#define HM330X_IIC_ADDR  0x40
#define SELECT_COMM_CMD   0X88
class HM330X_I2C {
  public:

    ErrorCode IIC_write_byte(uint8_t reg, uint8_t byte);

    ErrorCode IIC_read_byte(uint8_t reg, uint8_t* byte);

    void set_iic_addr(uint8_t IIC_ADDR);

    ErrorCode IIC_read_16bit(uint8_t start_reg, uint16_t* value);

    ErrorCode IIC_write_16bit(uint8_t reg, uint16_t value);

    ErrorCode IIC_read_bytes(uint8_t start_reg, uint8_t* data, uint32_t data_len);

    ErrorCode IIC_SEND_CMD(uint8_t CMD);

  private:
    uint8_t _IIC_ADDR;
};

class HM330X : public HM330X_I2C {
  public:
    HM330X(uint8_t IIC_ADDR = HM330X_IIC_ADDR);

    ErrorCode init();

    ErrorCode select_comm();

    ErrorCode read_sensor_value(uint8_t* data, uint32_t data_len);

    ErrorCode parse_result(uint8_t* dataIn, uint16_t* dataOut);

};


/*
 * ***********************************************************
   SGP30
 * ***********************************************************
*/

typedef enum
{
  SUCCESS = 0,
  ERR_BAD_CRC,
  ERR_I2C_TIMEOUT,
  SELF_TEST_FAIL
} SGP30ERR;

const uint8_t init_air_quality[2] = {0x20, 0x03};
const uint8_t measure_air_quality[2] = {0x20, 0x08};
const uint8_t get_baseline[2] = {0x20, 0x15};
const uint8_t set_baseline[2] = {0x20, 0x1E};
const uint8_t set_humidity[2] = {0x20, 0x61};
const uint8_t measure_test[2] = {0x20, 0x32};
const uint8_t get_feature_set_version[2] = {0x20, 0x2F};
const uint8_t get_serial_id[2] = {0x36, 0x82};
const uint8_t measure_raw_signals[2] = {0x20, 0x50};

class SGP30
{
  // user-accessible "public" interface
public:
  uint16_t CO2;
  uint16_t TVOC;
  uint16_t baselineCO2;
  uint16_t baselineTVOC;
  uint16_t featureSetVersion;
  uint16_t H2;
  uint16_t ethanol;
  uint64_t serialID;

  //default constructor
  SGP30();

  //Start I2C communication using specified port
  bool begin(TwoWire &wirePort = Wire); //If user doesn't specificy then Wire will be used

  //Initializes sensor for air quality readings
  void initAirQuality(void);

  //Measure air quality
  //Call in regular intervals of 1 second to maintain synamic baseline calculations
  //CO2 returned in ppm, Total Volatile Organic Compounds (TVOC) returned in ppb
  //Will give fixed values of CO2=400 and TVOC=0 for first 15 seconds after init
  //returns false if CRC8 check failed and true if successful
  SGP30ERR measureAirQuality(void);

  //Returns the current calculated baseline from
  //the sensor's dynamic baseline calculations
  //Save baseline periodically to non volatile memory
  //(like EEPROM) to restore after new power up or
  //after soft reset using setBaseline();
  //returns false if CRC8 check failed and true if successful
  SGP30ERR getBaseline(void);

  //Updates the baseline to a previous baseline
  //Should only use with previously retrieved baselines
  //to maintain accuracy
  void setBaseline(uint16_t baselineCO2, uint16_t baselineTVOC);

  //Set humidity
  //humidity value is a fixed point 8.8 bit number
  //Value should be absolute humidity from humidity sensor
  //default value 0x0F80 = 15.5g/m^3
  //minimum value 0x0001 = 1/256g/m^3
  //maximum value 0xFFFF = 255+255/256 g/m^3
  //sending 0x0000 resets to default and turns off humidity compensation
  void setHumidity(uint16_t humidity);

  //gives feature set version number (see data sheet)
  //returns false if CRC8 check failed and true if successful
  SGP30ERR getFeatureSetVersion(void);

  //Intended for part verification and testing
  //these raw signals are used as inputs to the onchip calibrations and algorithms
  SGP30ERR measureRawSignals(void);

  //Soft reset - not device specific
  //will reset all devices that support general call mode
  void generalCallReset(void);

  //readout of serial ID register can identify chip and verify sensor presence
  //returns false if CRC8 check failed and true if successful
  SGP30ERR getSerialID(void);

  //Sensor runs on chip self test
  //returns true if successful
  SGP30ERR measureTest(void);

private:
  //This stores the requested i2c port
  TwoWire *_i2cPort;

  //SGP30's I2C address
  const byte _SGP30Address = 0x58;

  //Generates CRC8 for SGP30 from lookup table
  uint8_t _CRC8(uint16_t twoBytes);

#ifdef SGP30_LOOKUP_TABLE
  //lookup table for CRC8  http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
  const uint8_t _CRC8LookupTable[16][16] = {
      {0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E},
      {0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D},
      {0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8},
      {0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB},
      {0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA, 0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13},
      {0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50},
      {0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95},
      {0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6},
      {0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54},
      {0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17},
      {0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B, 0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2},
      {0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91},
      {0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69},
      {0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93, 0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A},
      {0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF},
      {0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC}};
#endif
};

#endif
