/*
    testBLE-transmit.cpp
    Main Driver for SX1280 BLE transmiter

    Copyright (c) 2020 SISLAB, VNU.
    Website    : http://sis.uet.vnu.edu.vn/
    Author     : Ninh.D.H
    Create Time: June 25/11/2020
    Version    : 0.2
    Change Log :
*/

#include <SPI.h>
#include "SX128XLT.h"
#include "SX128XLT_BLE.h"
#include <string.h>
extern "C" {
  #include "sis_aes.h"
}
#include "Settings.h"

SX128XLT LT;
SXBLE BLE;

void TX_packet_is_OK();
void TX_packet_is_Error();
void RX_packet_is_OK();
void RX_packet_is_Error();
void receiveCallback();
void led_Flash(uint16_t flashes, uint16_t delaymS);

uint32_t RXpacketCount;
uint32_t errors;

uint8_t RXBUFFER[RXBUFFER_SIZE];                 // create the buffer that received packets are copied into
uint8_t RXPacketL;                               // stores length of packet received
int8_t  PacketRSSI;                              // stores RSSI of received packet
int8_t  PacketSNR;                               // stores signal to noise ratio (SNR) of received packet
uint8_t TXPacketL;
uint32_t TXPacketCount, startmS, endmS;

uint8_t IdBuff[2];

// AES CCM Data
unsigned char ccm_payload[3] = {'A', 'B', 'C'};
unsigned char ccm_cipher[RXBUFFER_SIZE + CCM_TAG_SIZE];
unsigned char ccm_plainText[RXBUFFER_SIZE];
unsigned char ccm_key[CCM_KEY_SIZE] = {0x40, 0x41, 0x42, 0x43,
			                           0x44, 0x45, 0x46, 0x47,
			                           0x48, 0x49, 0x4a, 0x4b,
			                           0x4c, 0x4d, 0x4e, 0x4f};

unsigned char ccm_nonce[3] = {0x00, 0x00, 0x01};
unsigned char ccm_adata[CCM_ADATA_SIZE] = {0x00};

/* SETUP */
void setup()
{
  pinMode(LED1, OUTPUT);
  led_Flash(2, 125);                             // two quick LED flashes to indicate program start

  Serial.begin(115200);
  SPI.begin();

  /*
    SPI beginTranscation is normally part of library routines, but if it is disabled in library
    a single instance is needed here, so uncomment the program line below
  */
  // SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  /* setup hardware pins used by device, then check if device is found */
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    led_Flash(2, 125);                           // two further quick LED flashes to indicate device found
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                         // long fast speed LED flash indicates device error
    }
  }

  /*
    The function call list below shows the complete setup for the LoRa device using the information defined in the
    Settings.h file.
  */

  /* Setup BLE */
  /*************************************************************************************************************************/
  LT.setMode(MODE_STDBY_RC);
  LT.setRegulatorMode(USE_LDO);
  LT.setPacketType(PACKET_TYPE_BLE);
  LT.setRfFrequency(Frequency, Offset);
  LT.setBufferBaseAddress(0, 0);
  LT.setModulationParams(BandwidthBitRate, ModulationIndex, BT);
  LT.setPacketParams(BLE_PAYLOAD_LENGTH_MAX_37_BYTES, BLE_CRC_3B, BLE_TEST_PACKET, BLE_WHITENING_DISABLE, NULL, NULL, NULL);
  LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);  //set for IRQ on TX done and timeout on DIO1
  LT.setSyncWord1(Access_Address);
  /*************************************************************************************************************************/

  if (SERIAL_DEBUG)
  {
    Serial.println();
    LT.printModemSettings();                     // reads and prints the configured modem settings, useful check
    Serial.println();
    LT.printOperatingSettings();                 // reads and prints the configured operating settings, useful check
    Serial.println();
    Serial.println();
    LT.printRegisters(0x900, 0x9FF);             // print contents of device registers
    Serial.println();
    Serial.println();
  }

  Serial.print(F("Transmitter ready"));
  Serial.println();
}


/* LOOP */
void loop()
{
  for (uint8_t id = 4; id <= 5; id++)
  {
    IdBuff[0] = id + '0';
    TXPacketL = sizeof(IdBuff);
    IdBuff[TXPacketL - 1] = '*';

    if (SERIAL_DEBUG)
    {
      Serial.print(TXpower);
      Serial.print(F("dBm "));
      Serial.print(F("Packet> "));
      LT.printASCIIPacket(IdBuff, TXPacketL);
    }

    digitalWrite(LED1, HIGH);
    startmS =  millis();
    if (LT.transmit(IdBuff, TXPacketL, 10000, TXpower, WAIT_TX))
    {
      endmS = millis();
      TXPacketCount++;
      TX_packet_is_OK();
      receiveCallback();
    }
    else
    {
      TX_packet_is_Error();
    }

    digitalWrite(LED1, LOW);
    delay(packet_delay);
  }
  delay(packet_delay);
}



void receiveCallback()
{
  //wait for a packet to arrive with 3seconds (3000mS) timeout
  RXPacketL = LT.receive(RXBUFFER, RXBUFFER_SIZE, 3000, WAIT_RX);
  digitalWrite(LED1, HIGH);

  PacketRSSI = LT.readPacketRSSI();
  PacketSNR = LT.readPacketSNR();

  if (RXPacketL == 0)
  {
    RX_packet_is_Error();
  }
  else
  {
    RX_packet_is_OK();
  }

  digitalWrite(LED1, LOW);
}

void TX_packet_is_OK()
{
  if (SERIAL_DEBUG)
  {
    uint16_t localCRC;
    Serial.print(F("  BytesSent,"));
    Serial.print(TXPacketL);
    localCRC = LT.CRCCCITT(IdBuff, TXPacketL, 0xFFFF);
    Serial.print(F("  CRC,"));
    Serial.print(localCRC, HEX);
    Serial.print(F("  TransmitTime,"));
    Serial.print(endmS - startmS);
    Serial.print(F("mS"));
    Serial.print(F("  PacketsSent,"));
    Serial.print(TXPacketCount);
  }
}


void TX_packet_is_Error()
{
  if (SERIAL_DEBUG)
  {
    uint16_t IRQStatus;
    IRQStatus = LT.readIrqStatus();
    Serial.print(F(" SendError,"));
    Serial.print(F("Length,"));
    Serial.print(TXPacketL);
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);
    LT.printIrqStatus();
  }
}

void RX_packet_is_OK()
{
  uint16_t IRQStatus, localCRC;
  IRQStatus = LT.readIrqStatus();
  RXpacketCount++;

  if (SERIAL_DEBUG)
  {
    Serial.println();
    Serial.print("RX   Packet > ");
  }
  // print the packet as JSON string
  uint8_t i = 0;
  while ((char)RXBUFFER[i] != '}')
  {
    Serial.print((char)RXBUFFER[i]);
    i++;
  }
  Serial.println ("}");

  // AES test
  if (AES_TEST)
  {
    aes_aead_ccm(ccm_cipher, ccm_key,
                 ccm_adata, sizeof(ccm_adata),
                 RXBUFFER, RXBUFFER_SIZE,
                 ccm_nonce, CCM_NONCE_SIZE,
                 CCM_TAG_SIZE, CCM, KEY_128);
    int8_t aesStatus = aes_aead_ccm_decrypt(ccm_plainText, ccm_key,
  		                                    ccm_adata, CCM_ADATA_SIZE,
   		                                    ccm_cipher, sizeof(ccm_cipher),
   		                                    ccm_nonce, CCM_NONCE_SIZE,
   		                                    CCM_TAG_SIZE, CCM, KEY_128);
    Serial.print("  Encrypt: ");    
    for (unsigned int i = 0; i < sizeof(ccm_cipher); i++)
    {
      Serial.print(ccm_cipher[i], HEX);
    }
    Serial.println();  
    Serial.print("  Decrypt: ");
    if (aesStatus == AES_OK)
    {
      for (unsigned int i = 0; i < sizeof(ccm_plainText); i++)
      {
        if (ccm_plainText[i] != 0)
        {
          Serial.print((char)ccm_plainText[i]);
        }
      }
      Serial.println();
    }
  }

  if (SERIAL_DEBUG)
  {
    // external CRC calculation of the RXBUFFE
    localCRC = LT.CRCCCITT(RXBUFFER, RXPacketL, 0xFFFF);
    Serial.print(F("RX Packet CRC,"));
    Serial.print(localCRC, HEX);
    Serial.print(F(",RSSI,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm,SNR,"));
    Serial.print(PacketSNR);
    Serial.print(F("dB,Length,"));
    Serial.print(RXPacketL);
    Serial.print(F(",Packets,"));
    Serial.print(RXpacketCount);
    Serial.print(F(",Errors,"));
    Serial.print(errors);
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);
    Serial.println();
    Serial.println();
  }
}


void RX_packet_is_Error()
{
  if (SERIAL_DEBUG)
  {
    uint16_t IRQStatus;
    IRQStatus = LT.readIrqStatus();

    if (IRQStatus & IRQ_RX_TIMEOUT)
    {
      Serial.print(F(" RXTimeout"));
    }
    else
    {
      errors++;
      Serial.print(F(" PacketError"));
      Serial.print(F(",RSSI,"));
      Serial.print(PacketRSSI);
      Serial.print(F("dBm,SNR,"));
      Serial.print(PacketSNR);
      Serial.print(F("dB,Length,"));
      Serial.print(LT.readRXPacketL());
      Serial.print(F(",Packets,"));
      Serial.print(RXpacketCount);
      Serial.print(F(",Errors,"));
      Serial.print(errors);
      Serial.print(F(",IRQreg,"));
      Serial.print(IRQStatus, HEX);
      LT.printIrqStatus();
    }
  }
  //gives a longer LED flash for error
  delay(250);
}

void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}

void BLEadv()
{
  LT.setRfFrequency(2426000000, Offset);
  BLE.setPhone(PHONE_ANDROID);
  BLE.setMAC(MY_MAC_0, MY_MAC_1, MY_MAC_2, MY_MAC_3, MY_MAC_4, MY_MAC_5);
  BLE.setName("SX1280");
  uint8_t dataBytes[] = "test";
  BLE.setData(&dataBytes, sizeof(dataBytes));
  BLE.blePacketEncode(BLE._packet, BLE._length, 38);

  uint8_t buff[BLE._length];
  for (uint8_t i = 0; i < BLE._length; i++)
  {
    buff[i] = BLE._packet[i];
  }

  TXPacketL = sizeof(buff);

  digitalWrite(LED1, HIGH);
  if (LT.transmit(buff, TXPacketL, 10000, TXpower, WAIT_TX))
  {
    if (SERIAL_DEBUG)
    {
      Serial.println("BLE OK");
    }
  }
  else
  {
    Serial.println("BLE ERROR");
  }
  digitalWrite(LED1, LOW);
  
  LT.setRfFrequency(Frequency, Offset);
}
