/*
    testSensor.ino
    Main Driver for ADENT4IOT sensors

    Copyright (c) 2020 SISLAB, VNU.
    Website    : http://sis.uet.vnu.edu.vn/
    Author     : Ninh.D.H
    Create Time: 13 Aug 2020
    Version    : 1.0
    Change Log :
*/

#include "sensor.h"
#include <Wire.h>
#include <Arduino.h>
#include <main.cpp>
#include <twi.h>


SGP30 sgp30;

ClosedCube_SHT31D sht3xd;

HM330X HM330;

uint16_t PMvalue[7];


int count = 0;

void setup() {

  Serial.begin(115200);


  Wire.begin();
  //Initialize sensor
  if (sgp30.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
    while (1);
  }
  //Initializes sensor for air quality readings
  //measureAirQuality should be called in one second increments after a call to initAirQuality
  sgp30.initAirQuality();

  sht3xd.begin(0x45); // I2C address: 0x44 or 0x45
	Serial.print("Serial #");
	Serial.println(sht3xd.readSerialNumber());
  if (HM330.init()) {
    Serial.println("HM330X init failed!!!");
  }


  delay(1000);
}

void loop() {

  Serial.print("#");
  Serial.println(count);


	SHT31D result = sht3xd.readTempAndHumidity(SHT3XD_REPEATABILITY_LOW, SHT3XD_MODE_CLOCK_STRETCH, 50);
  Serial.print("T=");
	Serial.print(result.t);
	Serial.print("C, RH=");
	Serial.print(result.rh);
	Serial.println("%");

  //First fifteen readings will be
  //CO2: 400 ppm  TVOC: 0 ppb
  //measure CO2 and TVOC levels
  sgp30.setHumidity(result.rh);
  sgp30.measureAirQuality();
  Serial.print("CO2: ");
  Serial.print(sgp30.CO2);
  Serial.print(" ppm\tTVOC: ");
  Serial.print(sgp30.TVOC);
  Serial.println(" ppb");

  uint8_t buf[30];

  HM330.read_sensor_value(buf, 29);
  HM330.parse_result(buf, PMvalue);

  Serial.print("PM1.0: ");
  Serial.print(PMvalue[4]);
  Serial.print(" ug/m3\tPM2.5: ");
  Serial.print(PMvalue[5]);
  Serial.print(" ug/m3\tPM10: ");
  Serial.print(PMvalue[6]);
  Serial.println(" ug/m3");

  Serial.println();

  count++;
  delay(1500); //Wait 1 second
}

