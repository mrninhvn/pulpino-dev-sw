
/* System Define */
#ifdef SISLAB_PULPINO
#define delay(a) delay(25*a)
#endif

#define SERIAL_DEBUG false
#define AES_TEST true

/* SX1280 Module Parameters */
#define NSS 10
#define RFBUSY 6
#define NRESET 3
#define LED1 8
#define DIO1 2
#define DIO2 -1         // not used
#define DIO3 -1         // not used
#define RX_EN -1        // pin for RX enable, used on some SX1280 devices, set to -1 if not used
#define TX_EN -1        // pin for TX enable, used on some SX1280 devices, set to -1 if not used
#define BUZZER -1       // connect a buzzer here if wanted
#define LORA_DEVICE DEVICE_SX1280

/* BLE Modem Parameters */
const uint32_t Frequency = 2436000000;                    // frequency of transmissions
const int32_t Offset = 0;                                 // offset frequency for calibration purposes

const uint8_t BandwidthBitRate = BLE_BR_1_000_BW_1_2;     // 1Mb/s
const uint8_t ModulationIndex = MOD_IND_1_5;              // 0.5
const uint8_t BT = BT_0_5;                                // 0.5
const uint32_t Access_Address = 0x8e89bed6;               // BLE Advertising

const int8_t TXpower  = 10;                               // power for transmissions in dBm
const uint16_t packet_delay = 1000;                       // mS delay between packets
#define RXBUFFER_SIZE 37                                  // RX buffer size

/* BLE MAC Address */
#define MY_MAC_0  'A'
#define MY_MAC_1  'A'
#define MY_MAC_2  'A'
#define MY_MAC_3  'A'
#define MY_MAC_4  'A'
#define MY_MAC_5  'A'

/* AES CCM Settings */
#define CCM_KEY_SIZE   		16
#define CCM_NONCE_SIZE 		3
#define CCM_ADATA_SIZE 		1
#define CCM_TAG_SIZE   		4
