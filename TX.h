#include <Adafruit_ADS1015.h>    //soligen2010 fork on github
#include "Config.h"


#ifndef __have__RC_RX_TX_TX_h__
#define __have__RC_RX_TX_TX_h__

#define TX_BUTTON_DOUBLE_CLICK_TIME 600
#define TX_BUTTON_HOLD_TIME         1200

#define ADC_VERSION               Adafruit_ADS1015   // change to Adafruit_ADS1015 for the 12-bit version, Adafruit_ADS1115 for the 16-bit version
#define ADC_SAMPLE_RATE           ADS1015_DR_3300SPS

/*
#define ADC_INPUT_LOW              {6450,4940,4250,6820}
#define ADC_INPUT_MID              {14120,13820,12880,14170}
#define ADC_INPUT_HIGH             {21150,21480,20920,21650}
*/
#define ADC_INPUT_LOW              {385,313,280,470}
#define ADC_INPUT_MID              {892,869,809,884}
#define ADC_INPUT_HIGH             {1335,1350,1312,1360}

#define STICK_DEAD_ZONE_RATIO      .03

#define THROTTLE_ADC_PIN           THROTTLE_CHANNEL
#define YAW_ADC_PIN                YAW_CHANNEL
#define ROLL_ADC_PIN               ROLL_CHANNEL
#define PITCH_ADC_PIN              PITCH_CHANNEL

#define AUX1_INPUT_PIN             2
#define AUX2_INPUT_PIN             3
#define AUX3_INPUT_PIN             4
#define AUX4_INPUT_PIN             5

#define BUTTON1_INPUT_PIN          6
#define BUTTON2_INPUT_PIN          7
#define BUTTON3_INPUT_PIN          8
#define BUTTON4_INPUT_PIN          9

#define TIMER_RESET_BUTTON_INPUT_PIN   BUTTON1_INPUT_PIN
#define MODEL_SELECT_BUTTON_INPUT_PIN  BUTTON2_INPUT_PIN
#define RATE_BUTTON_INPUT_PIN          BUTTON3_INPUT_PIN
#define TRIM_BUTTON_INPUT_PIN          BUTTON4_INPUT_PIN

#define BIND_MODE_BUTTON           BUTTON4_INPUT_PIN
#define POWER_LOW_BUTTON           BUTTON3_INPUT_PIN
#define POWER_HIGH_BUTTON          BUTTON2_INPUT_PIN
#define POWER_MAX_BUTTON           BUTTON1_INPUT_PIN

#define ANALOG_TRIM_BUTTON_PIN     A6
#define TRIM_BUTTON_STEP_SIZE      2

#define PPM_OUTPUT_PIN             A3

#define BUZZER_PIN                 15

enum buzzer_t { OFF, ON, PULSE_SLOW, PULSE_FAST };

#define VOLTAGE_SENSE_PIN          A2

#define LOW_VOLTAGE_THRESHOLD      732    // approx 7.55V measured
// below is incorrect.  MEasured at 7.77V
//#define LOW_VOLTAGE_THRESHOLD      752   // battery low is 7.6V, diode has .3 drop so target is 7.3V
                                        // with resistor divider value of 809 = 7.86V (tested)
                                        // so 7.3V = 752 (calculated)

  uint16_t mapValue (bool reverse, uint16_t inValue, uint16_t inLow, uint16_t inMid, uint16_t inHigh, uint16_t outLow, uint16_t outHigh, int16_t trimData, int8_t expoPercent);
  uint16_t mapThrottle (bool reverse, uint16_t inValue, uint16_t inLow, uint16_t inMid, uint16_t inHigh, uint16_t outLow, uint16_t outHigh, int16_t trimData, int8_t expoPercent);
  uint16_t mapChannelValue (int channel,uint16_t inValue);
  void setupTransmitter();
  void processTransmitter();
  void sendPacket();
  void sendPacket_V2();
  void sendPacket_V3();
  void sendPacket_V3_XMIT();
  void sendPacket_PPM();
  void sendPacket_Serial(uint32_t recieverProtocolNegative);
  void ppmSetup();
  void ppmDisable();
  bool getChannelValues ();
  uint16_t getChannelValueFromPin(bool& changed, uint16_t originalValue, uint8_t pin, int channel);
  uint16_t getChannelValueFromADC(bool& changed, uint16_t originalValue, ADC_VERSION adc, uint8_t adcPin, int channel);
  void timerService();
  void buzzerService();
  void setTrim();
  void reSetTrim();
  void setCurrentModel(uint8_t newModel);
  void checkButtons();
  uint16_t applyExpo (int8_t inExpoPercent, uint16_t inValue, uint16_t inLow, uint16_t inMid, uint16_t inHigh);
  void maintainTimer(uint16_t inValue, uint16_t inLow, int timerWarningSec, int timerAlertSec);
  char * TimeToString(unsigned long t);
  void senseVoltage(int sensePin);
  int findFirstStringDifference(char * first, char * second, int arrayLength);
  void copyString(char * first, char * second, int arrayLength);
  void applyMixes();
  void displayRateSetting();
  void changeWritingPipe(uint64_t newRadioPipeID);

#endif


