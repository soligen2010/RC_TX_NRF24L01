
#ifndef __have__RC_RX_TX_Model_Config_h__
#define __have__RC_RX_TX_Model_Config_h__

#include "Config.h"

#define NUM_MODELS        7  // models numbers start at 0, so the max model number is one less than this. The size of data in EEPROM constrains this.
#define MODEL_NAME_LENGTH 13  // name can only be 13 characters max
#define MODEL_MAX_MIXES   10  // Maximum numnber of mixes that can be defined per model
#define LAST_MIX_ENTRY    -1  // destChannel that signifies end of list if all MODEL_MAX_MIXES are not used

// Channel order    is pitch, roll, yaw, throttle, AUX1, AUX2, AUX3, AUX4
// Channel indexes are   0     1     2      3       4     5      6    7


// Positive expo is less sensitive center stick, negative is more sensitive


  #define MODEL2_NAME                          "FT22 Mini 800"
  #define MODEL2_CHANNEL_REVERSE_VALUES        {0,1,1,0,0,0,0,0}   // 0 for normal 1 for reverse  
  #define MODEL2_CHANNEL_LOW_OUT_VALUES        {1150,1250,1250,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,1250}
  #define MODEL2_CHANNEL_HIGH_OUT_VALUES       {1750,1750,1750,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,1750}
  #define MODEL2_EXPO_PERCENT                  {55,55,55,0,0,0,0,0}
  #define MODEL2_TIME_WARNING                  300   // seconds.  Zero is disabled
  #define MODEL2_TIME_ALERT                    330   // seconds  Zero is disabled
  #define MODEL2_MIX                           {ROLL_CHANNEL,PITCH_CHANNEL,75},{ROLL_CHANNEL,ROLL_CHANNEL,75},{AUX4_CHANNEL,ROLL_CHANNEL,75},{AUX4_CHANNEL,PITCH_CHANNEL,-75},{-1,-1,3}   // Dest, Source, Percent.  -1 for end of list
  #define MODEL2_HIGH_RATE_OFFSET              {150,150,150,0,0,0,0,0} // Amount to expand the high/low range out value range for high rates
  #define MODEL2_RECIEVER_VERSION              3  // 0= PPM Output, 1 = obsolete, 2 = obsolete; 3 = RC_RX_CABELL_V3_FHSS, big hex number is protocol for 4 in 1 multiprotocol module (first 4 bytes of packet)
  #define MODEL2_RECIEVER_OUTPUT_MODE          0  // 0= PWM (single channel PPM Output, 1 = SUM PPM
  #define MODEL2_CHANNELS_TO_SEND              8  // 4 to 16
  
  #define MODEL3_NAME                          "FT Arrow 800"
  #define MODEL3_CHANNEL_REVERSE_VALUES        {1,0,1,0,0,0,0,0}   // 0 for normal 1 for reverse  
  #define MODEL3_CHANNEL_LOW_OUT_VALUES        {1130,1110,1130,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE}
  #define MODEL3_CHANNEL_HIGH_OUT_VALUES       {1870,1890,1870,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE}
  #define MODEL3_EXPO_PERCENT                  {55,55,55,0,0,0,0,0}
  #define MODEL3_TIME_WARNING                  380   // seconds  Zero is disabled
  #define MODEL3_TIME_ALERT                    410   // seconds  Zero is disabled
  #define MODEL3_MIX                           {PITCH_CHANNEL,ROLL_CHANNEL,50},{ROLL_CHANNEL,ROLL_CHANNEL,50},{PITCH_CHANNEL,PITCH_CHANNEL,-50},{ROLL_CHANNEL,PITCH_CHANNEL,50},{-1,-1,3}   // Dest, Source, Percent.  -1 for end of list
  #define MODEL3_HIGH_RATE_OFFSET              {130,130,130,0,0,0,0,0} // Amount to expand the high/low range out value range for high rates
  #define MODEL3_RECIEVER_VERSION              3  // 0= PPM Output, 1 = obsolete, 2 = obsolete; 3 = RC_RX_CABELL_V3_FHSS, big hex number is protocol for 4 in 1 multiprotocol module (first 4 bytes of packet)
  #define MODEL3_RECIEVER_OUTPUT_MODE          0  // 0= PWM (single channel PPM Output, 1 = SUM PPM
  #define MODEL3_CHANNELS_TO_SEND              4  // 4 to 16

  #define MODEL1_NAME                          "FTTRNR SPRT"
  #define MODEL1_CHANNEL_REVERSE_VALUES        {1,1,1,0,0,0,0,0}   // 0 for normal 1 for reverse  
  #define MODEL1_CHANNEL_LOW_OUT_VALUES        {1200,1250,1200,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE}
  #define MODEL1_CHANNEL_HIGH_OUT_VALUES       {1800,1750,1800,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE}
  #define MODEL1_EXPO_PERCENT                  {48,48,48,0,0,0,0,0}
  #define MODEL1_TIME_WARNING                  410   // seconds  Zero is disabled
  #define MODEL1_TIME_ALERT                    520   // seconds  Zero is disabled
                                               // mix in some aux1 to elevator for inverted flight
  #define MODEL1_MIX                           {-1,-1,3}   // Dest, Source, Percent.  -1 for end of list
  #define MODEL1_HIGH_RATE_OFFSET              {50,50,50,0,0,0,0,0} // Amount to expand the high/low range out value range for high rates
  #define MODEL1_RECIEVER_VERSION              3  // 0= PPM Output, 1 = obsolete, 2 = obsolete; 3 = RC_RX_CABELL_V3_FHSS, big hex number is protocol for 4 in 1 multiprotocol module (first 4 bytes of packet)
  #define MODEL1_RECIEVER_OUTPUT_MODE          0  // 0= PWM (single channel PPM Output, 1 = SUM PPM
  #define MODEL1_CHANNELS_TO_SEND              16  // 4 to 16

  #define MODEL5_NAME                          "P-47"
  #define MODEL5_CHANNEL_REVERSE_VALUES        {1,1,1,0,0,0,0,0}   // 0 for normal 1 for reverse  
  #define MODEL5_CHANNEL_LOW_OUT_VALUES        {1200,1250,1200,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE}
  #define MODEL5_CHANNEL_HIGH_OUT_VALUES       {1800,1750,1800,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE}
  #define MODEL5_EXPO_PERCENT                  {48,48,48,0,0,0,0,0}
  #define MODEL5_TIME_WARNING                  410   // seconds  Zero is disabled
  #define MODEL5_TIME_ALERT                    520   // seconds  Zero is disabled
                                               // mix in some aux1 to elevator for inverted flight
  #define MODEL5_MIX                           {-1,-1,3}   // Dest, Source, Percent.  -1 for end of list
  #define MODEL5_HIGH_RATE_OFFSET              {50,50,50,0,0,0,0,0} // Amount to expand the high/low range out value range for high rates
  #define MODEL5_RECIEVER_VERSION              3  // 0= PPM Output, 1 = obsolete, 2 = obsolete; 3 = RC_RX_CABELL_V3_FHSS, big hex number is protocol for 4 in 1 multiprotocol module (first 4 bytes of packet)
  #define MODEL5_RECIEVER_OUTPUT_MODE          0  // 0= PWM (single channel PPM Output, 1 = SUM PPM
  #define MODEL5_CHANNELS_TO_SEND              8  // 4 to 16

  #define MODEL0_NAME                          "Serial Test"
  #define MODEL0_CHANNEL_REVERSE_VALUES        {1,0,1,0,0,0,0,0}   // 0 for normal 1 for reverse  
  #define MODEL0_CHANNEL_LOW_OUT_VALUES        {1200,1250,1200,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE}
  #define MODEL0_CHANNEL_HIGH_OUT_VALUES       {1800,1750,1800,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE}
  #define MODEL0_EXPO_PERCENT                  {48,48,48,0,0,0,0,0}
  #define MODEL0_TIME_WARNING                  0   // seconds  Zero is disabled
  #define MODEL0_TIME_ALERT                    0   // seconds  Zero is disabled
  #define MODEL0_MIX                           {-1,-1,3}   // Dest, Source, Percent.  -1 for end of list
  #define MODEL0_HIGH_RATE_OFFSET              {50,50,50,0,0,0,0,0} // Amount to expand the high/low range out value range for high rates
 // #define MODEL0_RECIEVER_VERSION              0x5401000C  // without telemetry
  #define MODEL0_RECIEVER_VERSION              0x5401100C  // with telemetry
 // #define MODEL0_RECIEVER_VERSION              0  // ppm
  #define MODEL0_RECIEVER_OUTPUT_MODE          0  // 0= PWM (single channel PPM Output, 1 = SUM PPM
  #define MODEL0_CHANNELS_TO_SEND              16  // 4 to 16

  #define MODEL4_NAME                          "BuddyBox"
  #define MODEL4_CHANNEL_REVERSE_VALUES        {0,0,0,0,0,0,0,0}   // 0 for normal 1 for reverse  
  #define MODEL4_CHANNEL_LOW_OUT_VALUES        {CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE}
  #define MODEL4_CHANNEL_HIGH_OUT_VALUES       {CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE}
  #define MODEL4_EXPO_PERCENT                  {0,0,0,0,0,0,0,0}
  #define MODEL4_TIME_WARNING                  0   // seconds  Zero is disabled
  #define MODEL4_TIME_ALERT                    0   // seconds  Zero is disabled
  #define MODEL4_MIX                           {-1,-1,3}   // Dest, Source, Percent.  -1 for end of list
  #define MODEL4_HIGH_RATE_OFFSET              {0,0,0,0,0,0,0,0} // Amount to expand the high/low range out value range for high rates
  #define MODEL4_RECIEVER_VERSION              0  // 0= PPM Output, 1 = obsolete, 2 = obsolete; 3 = RC_RX_CABELL_V3_FHSS, big hex number is protocol for 4 in 1 multiprotocol module (first 4 bytes of packet)
  #define MODEL4_RECIEVER_OUTPUT_MODE          0  // 0= PWM (single channel PPM Output, 1 = SUM PPM
  #define MODEL4_CHANNELS_TO_SEND              4  // 4 to 16

  #define MODEL6_NAME                          "E010 Serial"
  #define MODEL6_CHANNEL_REVERSE_VALUES        {0,0,1,0,1,1,1,1}   // 0 for normal 1 for reverse  
  #define MODEL6_CHANNEL_LOW_OUT_VALUES        {CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,1100,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE,CHANNEL_MIN_VALUE}
  #define MODEL6_CHANNEL_HIGH_OUT_VALUES       {CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE,CHANNEL_MAX_VALUE}
  #define MODEL6_EXPO_PERCENT                  {30,30,20,0,0,0,0,0}
  #define MODEL6_TIME_WARNING                  200   // seconds  Zero is disabled
  #define MODEL6_TIME_ALERT                    220   // seconds  Zero is disabled
  #define MODEL6_MIX                           {AUX1_CHANNEL,AUX4_CHANNEL,100},{-1,-1,3}   // Dest, Source, Percent.  -1 for end of list
  #define MODEL6_HIGH_RATE_OFFSET              {0,0,0,0,0,0,0,0} // Amount to expand the high/low range out value range for high rates
  #define MODEL6_RECIEVER_VERSION              0x55524400  // 0= PPM Output, 1 = obsolete, 2 = obsolete; 3 = RC_RX_CABELL_V3_FHSS, big hex number is protocol for 4 in 1 multiprotocol module (first 4 bytes of packet)
  #define MODEL6_RECIEVER_OUTPUT_MODE          0  // 0= PWM (single channel PPM Output, 1 = SUM PPM
  #define MODEL6_CHANNELS_TO_SEND              16  // 4 to 16


/////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
   char modelName [MODEL_NAME_LENGTH+1];
} modelNameType;

typedef struct {
   int8_t destChannel;
   int8_t sourceChannel;
   int8_t mixPercent;
} mixType;

const mixType mixItem [NUM_MODELS][MODEL_MAX_MIXES]        PROGMEM = {{MODEL0_MIX}
                                                                     ,{MODEL1_MIX}
                                                                     ,{MODEL2_MIX}
                                                                     ,{MODEL3_MIX}
                                                                     ,{MODEL4_MIX}
                                                                     ,{MODEL5_MIX}
                                                                     ,{MODEL6_MIX}
                                                                     };
const modelNameType modelNames [NUM_MODELS]                PROGMEM = {{MODEL0_NAME}
                                                                     ,{MODEL1_NAME}
                                                                     ,{MODEL2_NAME}
                                                                     ,{MODEL3_NAME}
                                                                     ,{MODEL4_NAME}
                                                                     ,{MODEL5_NAME}
                                                                     ,{MODEL6_NAME}
                                                                     };
const uint16_t channelLowOut   [NUM_MODELS] [NUM_CHANNELS] PROGMEM = {MODEL0_CHANNEL_LOW_OUT_VALUES
                                                                     ,MODEL1_CHANNEL_LOW_OUT_VALUES
                                                                     ,MODEL2_CHANNEL_LOW_OUT_VALUES
                                                                     ,MODEL3_CHANNEL_LOW_OUT_VALUES
                                                                     ,MODEL4_CHANNEL_LOW_OUT_VALUES
                                                                     ,MODEL5_CHANNEL_LOW_OUT_VALUES
                                                                     ,MODEL6_CHANNEL_LOW_OUT_VALUES
                                                                     }; 
const uint16_t channelHighOut  [NUM_MODELS] [NUM_CHANNELS] PROGMEM = {MODEL0_CHANNEL_HIGH_OUT_VALUES  
                                                                     ,MODEL1_CHANNEL_HIGH_OUT_VALUES
                                                                     ,MODEL2_CHANNEL_HIGH_OUT_VALUES
                                                                     ,MODEL3_CHANNEL_HIGH_OUT_VALUES
                                                                     ,MODEL4_CHANNEL_HIGH_OUT_VALUES
                                                                     ,MODEL5_CHANNEL_HIGH_OUT_VALUES
                                                                     ,MODEL6_CHANNEL_HIGH_OUT_VALUES
                                                                     };
const uint16_t channelHighRateOffset  [NUM_MODELS] [NUM_CHANNELS] PROGMEM = {MODEL0_HIGH_RATE_OFFSET  
                                                                            ,MODEL1_HIGH_RATE_OFFSET
                                                                            ,MODEL2_HIGH_RATE_OFFSET
                                                                            ,MODEL3_HIGH_RATE_OFFSET
                                                                            ,MODEL4_HIGH_RATE_OFFSET
                                                                            ,MODEL5_HIGH_RATE_OFFSET
                                                                            ,MODEL6_HIGH_RATE_OFFSET
                                                                     };
const bool     channelReverse  [NUM_MODELS] [NUM_CHANNELS] PROGMEM = {MODEL0_CHANNEL_REVERSE_VALUES 
                                                                     ,MODEL1_CHANNEL_REVERSE_VALUES
                                                                     ,MODEL2_CHANNEL_REVERSE_VALUES
                                                                     ,MODEL3_CHANNEL_REVERSE_VALUES
                                                                     ,MODEL4_CHANNEL_REVERSE_VALUES
                                                                     ,MODEL5_CHANNEL_REVERSE_VALUES
                                                                     ,MODEL6_CHANNEL_REVERSE_VALUES
                                                                     };
const int8_t channelExpoPercent [NUM_MODELS] [NUM_CHANNELS] PROGMEM = {MODEL0_EXPO_PERCENT 
                                                                      ,MODEL1_EXPO_PERCENT
                                                                      ,MODEL2_EXPO_PERCENT
                                                                      ,MODEL3_EXPO_PERCENT
                                                                      ,MODEL4_EXPO_PERCENT
                                                                      ,MODEL5_EXPO_PERCENT
                                                                      ,MODEL6_EXPO_PERCENT
                                                                      };
const int timeWarning  [NUM_MODELS]  PROGMEM =                       {MODEL0_TIME_WARNING 
                                                                     ,MODEL1_TIME_WARNING
                                                                     ,MODEL2_TIME_WARNING
                                                                     ,MODEL3_TIME_WARNING
                                                                     ,MODEL4_TIME_WARNING
                                                                     ,MODEL5_TIME_WARNING
                                                                     ,MODEL6_TIME_WARNING
                                                                     };
const int timeAlert  [NUM_MODELS]    PROGMEM =                       {MODEL0_TIME_ALERT
                                                                     ,MODEL1_TIME_ALERT
                                                                     ,MODEL2_TIME_ALERT
                                                                     ,MODEL3_TIME_ALERT
                                                                     ,MODEL4_TIME_ALERT
                                                                     ,MODEL5_TIME_ALERT
                                                                     ,MODEL6_TIME_ALERT
                                                                     };
const uint32_t recieverVersion  [NUM_MODELS]    PROGMEM =            {MODEL0_RECIEVER_VERSION
                                                                     ,MODEL1_RECIEVER_VERSION
                                                                     ,MODEL2_RECIEVER_VERSION
                                                                     ,MODEL3_RECIEVER_VERSION
                                                                     ,MODEL4_RECIEVER_VERSION
                                                                     ,MODEL5_RECIEVER_VERSION
                                                                     ,MODEL6_RECIEVER_VERSION
                                                                     };
const uint8_t recieverOutputMode  [NUM_MODELS]    PROGMEM =          {MODEL0_RECIEVER_OUTPUT_MODE
                                                                     ,MODEL1_RECIEVER_OUTPUT_MODE
                                                                     ,MODEL2_RECIEVER_OUTPUT_MODE
                                                                     ,MODEL3_RECIEVER_OUTPUT_MODE
                                                                     ,MODEL4_RECIEVER_OUTPUT_MODE
                                                                     ,MODEL5_RECIEVER_OUTPUT_MODE
                                                                     ,MODEL6_RECIEVER_OUTPUT_MODE
                                                                     };
const uint8_t channelsToXmit  [NUM_MODELS]    PROGMEM =              {MODEL0_CHANNELS_TO_SEND
                                                                     ,MODEL1_CHANNELS_TO_SEND
                                                                     ,MODEL2_CHANNELS_TO_SEND
                                                                     ,MODEL3_CHANNELS_TO_SEND
                                                                     ,MODEL4_CHANNELS_TO_SEND
                                                                     ,MODEL5_CHANNELS_TO_SEND
                                                                     ,MODEL6_CHANNELS_TO_SEND
                                                                     };
#endif
