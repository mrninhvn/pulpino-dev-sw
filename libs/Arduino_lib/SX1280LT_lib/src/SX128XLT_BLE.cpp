/*
    SX128LT_BLE.cpp
    Driver for SX1280 BLE mode

    Copyright (c) 2020 SISLAB, VNU.
    Website    : http://sis.uet.vnu.edu.vn/
    Author     : Ninh.D.H
    Create Time: June 2020
    Version    : 0.1
    Change Log :
*/

#include "SX128XLT_BLE.h"
#include <Arduino.h>

const byte SXBLE::chLe[] = {37,38,39} ;

SXBLE::SXBLE(){}

void SXBLE::setPhone(uint8_t phone_type){
  //byte no.0 PDU
  _packet[0] = phone_type; 
}
void SXBLE::setMAC(uint8_t m0, uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4, uint8_t m5){
  //length of payload is entered in byte no.1
  _length = 2;
  _packet[_length++] = m0;
  _packet[_length++] = m1;
  _packet[_length++] = m2;
  _packet[_length++] = m3;
  _packet[_length++] = m4;
  _packet[_length++] = m5;
  //length should be 8 by now
  //flags (LE-only, limited discovery mode)
  _packet[_length++] = 2;    //flag length
  _packet[_length++] = 0x01; //data type
  _packet[_length++] = 0x05; //actual flag
}

void SXBLE::setName(const char* name){
  // name must be set only once 
  //8,9,10 bytes are for flags 
  //name field starts from 11th byte
#if DEBUG == 1
  Serial.print(name);
  Serial.print(" ");
  Serial.println(strlen(name));
#endif
  if (strlen(name) != 0){
    //length of name including the terminating null character
    _packet[_length++] = strlen(name) + 1;
    _packet[_length++] = 0x08;//name type short name
    for (uint8_t i = 0; i < strlen(name); i++){
      _packet[_length++] = name[i];
    }
  }
  //else 
    //no name to be sent 
    //directly send manufacturere specific data 0xFF
    //do nothing here
}

void SXBLE::setData(const void* data,uint8_t dataLen){
  _dataFieldStartPoint = _dataFieldStartPoint==0?_length:_dataFieldStartPoint;
  _length = _dataFieldStartPoint;
  const uint8_t* current = reinterpret_cast<const uint8_t*>(data);

#if DEBUG == 1 
  Serial.print("data "); Serial.println(dataLen);
#endif
  _packet[_length++] = dataLen +1;
  _packet[_length++] = 0xFF;//data type
  for (uint8_t i = 0; i < dataLen; i++){
    _packet[_length++] = *(current);
    current++;
  }
  //CRC is appended to the data
  //CRC starting val 0x555555 acc. to spec
  _packet[_length++] = 0x55;
  _packet[_length++] = 0x55;
  _packet[_length++] = 0x55;
  _packet[1] = _length-5;
}

void SXBLE::BLEcrc(const uint8_t* data, uint8_t dataLen, uint8_t* outputCRC){
  // calculating the CRC based on a LFSR
  uint8_t i, temp, tempData;

  while (dataLen--){
    tempData = *data++;
    for (i = 0; i < 8; i++, tempData >>= 1){
      temp = outputCRC[0] >> 7;
      
      outputCRC[0] <<= 1;
      if (outputCRC[1] & 0x80){ outputCRC[0] |= 1; }
      outputCRC[1] <<= 1;
      if (outputCRC[2] & 0x80){ outputCRC[1] |= 1; }
      outputCRC[2] <<= 1;

      if (temp != (tempData & 1)){
        outputCRC[2] ^= 0x5B;
        outputCRC[1] ^= 0x06;
      }
    }
  }
}

uint8_t SXBLE::checkCRC(uint8_t *input,uint8_t length){
  uint8_t CRC[3] ={0x55,0x55,0x55 }; //initial value for bluetooth crc
  uint8_t dataLen = length - 3;
  BLEcrc(input, dataLen, CRC);
  if (CRC[0] == *(input + dataLen++) && CRC[1] == *(input + dataLen++) && CRC[2] == *(input + dataLen)){
    //PACKET IS VALID
    //Serial.println("VALID");
    return 1;
  } else {
    //PACKET is invalid
    //Serial.println("CORRUPT");
    return 0;
  }
}

void SXBLE::bleWhiten(uint8_t* data, uint8_t len, uint8_t whitenCoeff){
  // Implementing whitening with LFSR
  uint8_t  m;
  while (len--){
    for (m = 1; m; m <<= 1){
      if (whitenCoeff & 0x80){
        whitenCoeff ^= 0x11;
        (*data) ^= m;
      }
      whitenCoeff <<= 1;
    }
    data++;
  }
}
void SXBLE::blePacketEncode(uint8_t* packet, uint8_t len, uint8_t chan){
  // Assemble the packet to be transmitted
  // Packet length includes pre-populated crc
  uint8_t i, dataLen = len - 3;
  BLEcrc(packet, dataLen, packet + dataLen);
  for (i = 0; i < 3; i++, dataLen++)
    packet[dataLen] = reverseBits(packet[dataLen]);
  bleWhiten(packet, len, bleWhitenStart(chan));
}

void SXBLE::printPacket(){
  for (uint8_t i = 0; i < _length; i++){
    Serial.print("0x");
    Serial.print(_packet[i], HEX);
    Serial.print(",");
  }
  Serial.println();
}
