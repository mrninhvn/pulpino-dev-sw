/*
    SX128LT_BLE.h
    Driver for SX1280 BLE mode

    Copyright (c) 2020 SISLAB, VNU.
    Website    : http://sis.uet.vnu.edu.vn/
    Author     : Ninh.D.H
    Create Time: June 2020
    Version    : 0.1
    Change Log :
*/

#ifndef __SX128XLT_BLE_H__
#define __SX128XLT_BLE_H__

#include <Arduino.h>

#define DEBUG 0
#define PHONE_ANDROID   0x42
#define PHONE_IPHONE    0x40

class SXBLE{

private:

protected:
  void  bleWhiten(uint8_t* data, uint8_t len, uint8_t whitenCoeff);
  void  BLEcrc(const uint8_t* data, uint8_t len, uint8_t* output);
  uint8_t checkCRC(uint8_t *input,uint8_t length);

public:

  static const byte chLe[];
  uint8_t _packet[34]; //maximum size of payload 34
  uint8_t _length = 0; //length of packet filled
  uint8_t _dataFieldStartPoint = 0; 
  
  SXBLE();
  void setMAC(uint8_t m0, uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4, uint8_t m5);
  void setPhone(uint8_t phone_type);
  void setName(const char* name);
  void setData(const void* data,uint8_t dataLen);
  void  blePacketEncode(uint8_t* packet, uint8_t len, uint8_t chan);
  void printPacket();

  static uint8_t  reverseBits(uint8_t input){
    // reverse the bit order in a single byte
    uint8_t temp = 0;
    if (input & 0x80) temp |= 0x01;
    if (input & 0x40) temp |= 0x02;
    if (input & 0x20) temp |= 0x04;
    if (input & 0x10) temp |= 0x08;
    if (input & 0x08) temp |= 0x10;
    if (input & 0x04) temp |= 0x20;
    if (input & 0x02) temp |= 0x40;
    if (input & 0x01) temp |= 0x80;
    return temp;
  }
  static inline uint8_t bleWhitenStart(uint8_t chan){
    //use left shifted one
    return reverseBits(chan) | 2;
  }

};

#endif
