
#include <Adafruit_ADS1015.h>    //soligen2010 fork on github
#include <RF24.h>

#ifndef __have__RC_RX_TX_Config_h__
#define __have__RC_RX_TX_Config_h__


#define NUM_CHANNELS            8

#define RADIO_ADDR              0xA4B7C123F4LL;   // CHANGE THIS TO SOMETHING UNIQUE

#define CABELL_BIND_RADIO_ADDR  0xA4B7C123F7LL

#define CABELL_NUM_CHANNELS     16                  // The maximum number of RC channels that can be sent in one packet
#define CABELL_MIN_CHANNELS     4                   // The minimum number of channels that must be included in a packet, the number of channels cannot be reduced any further than this
#define CABELL_PAYLOAD_BYTES    24                  // 12 bits per value * 16 channels

#define CABELL_RADIO_CHANNELS         9                  // This is 1/5 of the total number of radio channels used for FHSS 
#define CABELL_RADIO_MIN_CHANNEL_NUM  3                   // Channel 0 is right on the boarder of allowed frequency range, so move up to avoid bleeding over

#define CABELL_OPTION_MASK_CHANNEL_REDUCTION     0x0F
#define CABELL_OPTION_MASK_RECIEVER_OUTPUT_MODE  0x70
#define CABELL_OPTION_SHIFT_RECIEVER_OUTPUT_MODE 4
#define CABELL_OPTION_MASK_MAX_POWER_OVERRIDE    0x40

typedef struct {
   enum RxMode_t : uint8_t {   
         normal                 = 0,
         bind                   = 1,
         setFailSafe            = 2,
         normalWithTelemetry    = 3,
         telemetryResponse      = 4,
         unBind                 = 127
   } RxMode;
   uint8_t  reserved = 0;  // contains channel packet sent on
   uint8_t  option;
                          /*   mask 0x0F    : Channel reduction.  The number of channels to not send (subtracted frim the 16 max channels) at least 4 are always sent
                           *   mask 0x30>>4 : Reciever outout mode
                           *                  0 (00) = Single PPM on individual pins for each channel 
                           *                  1 (01) = SUM PPM on channel 1 pin
                           *                  2 (10) = Future use.  Reserved for SBUS output
                           *                  3 (11) = Unused
                           *   mask 0x40>>6   Contains max power override flag for Multiprotocol TX module. Also sent to RX
                           *   mask 0x80>>7   Unused 
                           */  
   uint8_t  modelNum;
   uint8_t  checkSum_LSB; 
   uint8_t  checkSum_MSB; 
   uint8_t  payloadValue [CABELL_PAYLOAD_BYTES] = {0}; //12 bits per channel value, unsigned
} CABELL_RxTxPacket_t;   

  #define RADIO_CE_PIN              10
  #define RADIO_CSN_PIN             14
  
  #define CHANNEL_MIN_VALUE         1000
  #define CHANNEL_MAX_VALUE         2000
  #define CHANNEL_MID_VALUE         ((CHANNEL_MIN_VALUE + CHANNEL_MAX_VALUE)/2)

  #define PITCH_CHANNEL             0
  #define ROLL_CHANNEL              1
  #define YAW_CHANNEL               2
  #define THROTTLE_CHANNEL          3
  #define AUX1_CHANNEL              4
  #define AUX2_CHANNEL              5
  #define AUX3_CHANNEL              6
  #define AUX4_CHANNEL              7


#endif
