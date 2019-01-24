#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>    //soligen2010 fork on github
#include "Config.h"
#include "Model_Config.h"
#include "TX.h"
#include <ClickEncoder.h>        //soligen2010 fork on github
#include <EEPROM.h>
#include <MsTimer2.h>
#include <LiquidCrystal_I2C.h>
#include "MySoftwareSerial.h"
#include "Rx_Tx_Util.h"

#include "SUM_PPM.h"      

#define EN_PIN     2
#define RW_PIN     1
#define RS_PIN     0
#define D4_PIN     4
#define D5_PIN     5
#define D6_PIN     6
#define D7_PIN     7
#define BL_PIN     3   //Backlight

LiquidCrystal_I2C lcd(0x27,   // Set the LCD I2C address
                      EN_PIN,
                      RW_PIN,
                      RS_PIN,
                      D4_PIN,
                      D5_PIN,
                      D6_PIN,
                      D7_PIN,
                      BL_PIN,
                      POSITIVE);  

// A7 is non-functional, used as here because it is the only unused pin and data is only transmitted
MySoftwareSerial outputSerial(A7, PPM_OUTPUT_PIN); // RX, TX

const char LCD_blankLine[]    = "                ";
const char LCD_blankHalfLine[] = "        ";
                      
RF24 radio(RADIO_CE_PIN,RADIO_CSN_PIN);
  
uint64_t radioPipeID;
const uint64_t radioNormalTxPipeID = RADIO_ADDR;

uint8_t radioChannel[CABELL_RADIO_CHANNELS];    
  
uint16_t channelValues [CABELL_NUM_CHANNELS] = {1500,1500,1500,1000,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500}; 

ADC_VERSION ads;  /*Define the Analog to Digital converter*/

struct modelData {
  int16_t trimData [NUM_CHANNELS] = {0,0,0,0,0,0,0,0};   // must save trim once with neutral sticks to initialize EEPROM
} modelData;

uint8_t currentModel = 0;
int16_t highRate = 0;         // 0 = low rates 1 = high rates

#define currentModelEEPROMAddress   0 

#define trimDataEEPROMBaseAddress   16    // leave first 16 bytes for current model and future use
#define modelDataEEPROMAddress (trimDataEEPROMBaseAddress + (currentModel * sizeof(modelData)))

ClickEncoder timerResetButton   = DigitalButton(TIMER_RESET_BUTTON_INPUT_PIN,false);
ClickEncoder modelSelectButton  = DigitalButton(MODEL_SELECT_BUTTON_INPUT_PIN,false);
ClickEncoder rateButton         = DigitalButton(RATE_BUTTON_INPUT_PIN,false);
ClickEncoder trimButton         = DigitalButton(TRIM_BUTTON_INPUT_PIN,false);

ClickEncoder trimPitchUpButton    = AnalogButton(ANALOG_TRIM_BUTTON_PIN,671,691);
ClickEncoder trimPitchDownButton  = AnalogButton(ANALOG_TRIM_BUTTON_PIN,757,777);
ClickEncoder trimRollLeftButton   = AnalogButton(ANALOG_TRIM_BUTTON_PIN,809,829);
ClickEncoder trimRollRightButton  = AnalogButton(ANALOG_TRIM_BUTTON_PIN,501,521);
ClickEncoder trimYawLeftButton    = AnalogButton(ANALOG_TRIM_BUTTON_PIN,867,887);
ClickEncoder trimYawRightButton   = AnalogButton(ANALOG_TRIM_BUTTON_PIN,843,863);

buzzer_t buzzerState = OFF;

bool bindMode = false;            // when true send bind command to cause reciever to bind to the current model number
bool setFailSafeValues = false;   // when true send set failsafe command to cause reciever save fail safe values (V3+)
bool modelLocked = true;
bool telemetryEnabled = false;

unsigned long timerElapsed = 0;

// volatile variables used to send info to V3 xmit process in interrupt
volatile bool     useV3Xmit        = false;
volatile bool     newV3PacketReady = false;
volatile uint8_t  V3SendBuffer[32] = {0};
volatile uint8_t  V3PacketSize = 32;

//------------------------------------------------------------------------------------------------------------------------
uint16_t mapChannelValue (int channel,uint16_t inValue) {
  if (channel == THROTTLE_CHANNEL) {
    return mapThrottle (pgm_read_byte( &channelReverse[currentModel][channel]),
                     inValue,
                     CHANNEL_MIN_VALUE,
                     CHANNEL_MID_VALUE,
                     CHANNEL_MAX_VALUE,
                     pgm_read_word(&channelLowOut [currentModel][channel]) - (highRate * pgm_read_word(&channelHighRateOffset [currentModel][channel])),
                     pgm_read_word(&channelHighOut[currentModel][channel]) + (highRate * pgm_read_word(&channelHighRateOffset [currentModel][channel])),
                     modelData.trimData[channel],
                     pgm_read_byte(&channelExpoPercent[currentModel][channel]));
  } else {  
    return mapValue (pgm_read_byte( &channelReverse[currentModel][channel]),
                     inValue,
                     CHANNEL_MIN_VALUE,
                     CHANNEL_MID_VALUE,
                     CHANNEL_MAX_VALUE,
                     pgm_read_word(&channelLowOut [currentModel][channel]) - (highRate * pgm_read_word(&channelHighRateOffset [currentModel][channel])),
                     pgm_read_word(&channelHighOut[currentModel][channel]) + (highRate * pgm_read_word(&channelHighRateOffset [currentModel][channel])),
                     modelData.trimData[channel],
                     pgm_read_byte(&channelExpoPercent[currentModel][channel]));
  }
}

//------------------------------------------------------------------------------------------------------------------------
uint16_t mapValue (bool reverse, uint16_t inValue, uint16_t inLow, uint16_t inMid, uint16_t inHigh, uint16_t outLow, uint16_t outHigh, int16_t trimData, int8_t expoPercent) {

//  uint16_t outValue = inValue; //(((int16_t) inValue) + ((reverse) ? (trimData * -1) : trimData));

  uint16_t outMid = inMid;
//  uint16_t outMid = (outLow + outHigh) / 2;

  uint16_t outValue = applyExpo (expoPercent, inValue, inLow, inMid, inHigh);
  outValue = constrain(outValue, inLow, inHigh);
  if ( outValue < inMid ) {
    outValue = map(outValue, inLow, inMid, outLow, outMid);
  } else {
    outValue = map(outValue, inMid, inHigh, outMid, outHigh);
  }
  if (reverse) {
    if (outValue > outMid) {
      outValue = outMid - (outValue - outMid);
    } else {
      outValue = outMid + (outMid - outValue);
    }
  }
  outValue = constrain(outValue + trimData,outLow,outHigh);
  outValue = constrain(outValue,inLow,inHigh);
  return outValue;
}

//------------------------------------------------------------------------------------------------------------------------
uint16_t mapThrottle (bool reverse, uint16_t inValue, uint16_t inLow, uint16_t inMid, uint16_t inHigh, uint16_t outLow, uint16_t outHigh, int16_t trimData, int8_t expoPercent) {

//  uint16_t outValue = inValue; //(((int16_t) inValue) + ((reverse) ? (trimData * -1) : trimData));

  uint16_t outMid = inMid;
//  uint16_t outMid = (outLow + outHigh) / 2;

  uint16_t outValue = applyExpo (expoPercent, inValue, inLow, inMid, inHigh);
  outValue = constrain(outValue, inLow, inHigh);
  if ( outValue < inMid ) {
    outValue = map(outValue, inLow, inMid, inLow, outMid);   // outLow changed to inLow.  Outlow will be handled at the end
  } else {
    outValue = map(outValue, inMid, inHigh, outMid, outHigh);
  }
  if (reverse) {
    if (outValue > outMid) {
      outValue = outMid - (outValue - outMid);
    } else {
      outValue = outMid + (outMid - outValue);
    }
  }
//  Serial.print(outValue);Serial.print("\t");
  outValue = constrain(outValue + trimData,inLow,outHigh);  // outLow changed to inLow.  Outlow will be handled at the end
  outValue = constrain(outValue,inLow,inHigh);
//  Serial.print(outValue);Serial.print("\t");
  if (outValue != inLow) outValue = map(outValue,inLow,inHigh,outLow,outHigh);
//  Serial.println(outValue);
  return outValue;
}

//------------------------------------------------------------------------------------------------------------------------
void setupTransmitter() {

  rf24_datarate_e radioDataRate = RF24_250KBPS ;  // RF24_1MBPS RF24_2MBPS RF24_250KBPS

  if (digitalRead(BIND_MODE_BUTTON) == LOW) {
    bindMode = true;
    modelLocked = false;
    radioPipeID = CABELL_BIND_RADIO_ADDR;
  }
  else
  {
    bindMode = false;
    modelLocked = true;
    radioPipeID = radioNormalTxPipeID;
    
  }

  getChannelSequence (radioChannel, CABELL_RADIO_CHANNELS, radioPipeID);

  outputSerial.setParity(PARITY_EVEN);
  outputSerial.setStopBits(1);
  outputSerial.setDataBits(8);
  outputSerial.begin(100000);     // Initialize serial output for Multiprotocol 4 in 1 module

  pinMode(AUX1_INPUT_PIN, INPUT_PULLUP);
  pinMode(AUX2_INPUT_PIN, INPUT_PULLUP);
  pinMode(AUX3_INPUT_PIN, INPUT_PULLUP);
  pinMode(AUX4_INPUT_PIN, INPUT_PULLUP);
  
//  pinMode(BUTTON1_INPUT_PIN, INPUT_PULLUP);
//  pinMode(BUTTON2_INPUT_PIN, INPUT_PULLUP);
//  pinMode(BUTTON3_INPUT_PIN, INPUT_PULLUP);
//  pinMode(BUTTON4_INPUT_PIN, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite (BUZZER_PIN,LOW);

  rateButton.setButtonHeldEnabled(true);
  rateButton.setDoubleClickEnabled(false);
  rateButton.setDoubleClickTime(TX_BUTTON_DOUBLE_CLICK_TIME);
  rateButton.setHoldTime(TX_BUTTON_HOLD_TIME);
  
  timerResetButton.setButtonHeldEnabled(true);
  timerResetButton.setDoubleClickEnabled(true);
  timerResetButton.setDoubleClickTime(TX_BUTTON_DOUBLE_CLICK_TIME);
  timerResetButton.setHoldTime(TX_BUTTON_HOLD_TIME);
  
  modelSelectButton.setButtonHeldEnabled(true);
  modelSelectButton.setDoubleClickEnabled(true);
  modelSelectButton.setDoubleClickTime(TX_BUTTON_DOUBLE_CLICK_TIME);
  modelSelectButton.setHoldTime(TX_BUTTON_HOLD_TIME);
  
  trimButton.setButtonHeldEnabled(true);
  trimButton.setDoubleClickEnabled(true);
  trimButton.setDoubleClickTime(TX_BUTTON_DOUBLE_CLICK_TIME);
  trimButton.setHoldTime(TX_BUTTON_HOLD_TIME);

  trimPitchUpButton.setButtonHeldEnabled(false);
  trimPitchUpButton.setDoubleClickEnabled(false);

  trimPitchDownButton.setButtonHeldEnabled(false);
  trimPitchDownButton.setDoubleClickEnabled(false);

  trimRollLeftButton.setButtonHeldEnabled(false);
  trimRollLeftButton.setDoubleClickEnabled(false);

  trimRollRightButton.setButtonHeldEnabled(false);
  trimRollRightButton.setDoubleClickEnabled(false);

  trimYawLeftButton.setButtonHeldEnabled(false);
  trimYawLeftButton.setDoubleClickEnabled(false);

  trimYawRightButton.setButtonHeldEnabled(false);
  trimYawRightButton.setDoubleClickEnabled(false);

  lcd.begin(16, 2);
  lcd.clear();
  
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setSPS(ADC_SAMPLE_RATE); 
  ads.begin();

  radio.begin();

  lcd.setCursor(0,0);
  if (bindMode) {
    lcd.print(F("Bind Mode    "));
    radio.setPALevel(RF24_PA_HIGH);  
  }
  else if (digitalRead(BUTTON1_INPUT_PIN) == LOW) {
    lcd.print(F("Power HIGH    "));
    radio.setPALevel(RF24_PA_HIGH);
  }
  else if (digitalRead(BUTTON2_INPUT_PIN) == LOW) {
    lcd.print(F("Power LOW    "));
    radio.setPALevel(RF24_PA_LOW);
  }
  else if (digitalRead(BUTTON3_INPUT_PIN) == LOW) {
    lcd.print(F("Power MIN     "));
    radio.setPALevel(RF24_PA_MIN);
  }
  else  {
    lcd.print(F("Power MAX     "));
    radio.setPALevel(RF24_PA_MAX);
  }

//  if(analogRead(ANALOG_TRIM_BUTTON_PIN) < 835) {
//    radioDataRate = RF24_1MBPS; 
//    telemetryEnabled = false;
//    lcd.setCursor(14,0);
//    lcd.print(F("1M"));
//  } else if(analogRead(ANALOG_TRIM_BUTTON_PIN) < 1020) {
//    radioDataRate = RF24_1MBPS; 
//    telemetryEnabled = true;
//    lcd.setCursor(14,0);
//    lcd.print(F("TE"));
//  }

  lcd.setCursor(0,1);
  lcd.print(F("Release Buttons"));

  while ((analogRead(ANALOG_TRIM_BUTTON_PIN) < 1020) ||(digitalRead(BUTTON1_INPUT_PIN) == LOW) || (digitalRead(BUTTON2_INPUT_PIN) == LOW) || (digitalRead(BUTTON3_INPUT_PIN) == LOW) || (digitalRead(BUTTON4_INPUT_PIN) == LOW)) {  }  // wait for all buttons to be released
  lcd.clear();
  
  radio.setChannel(1);               //This will get changed as soon as we start looping
  radio.setDataRate(radioDataRate );
  radio.setAutoAck(0);
  radio.openWritingPipe(radioPipeID);

  EEPROM.get(currentModelEEPROMAddress,currentModel);
  setCurrentModel(currentModel);  // Call this to be sure all logic for changing models is executed
  
  for (int x = 0;x < 8;x++) Serial.println(modelData.trimData[x]);

  
  MsTimer2::set(3,timerService); // every 3 milliseconds
  MsTimer2::start(); 

  lcd.setCursor(0,0);
  if (bindMode) {
    lcd.print(F("Bind    "));
  } else {
    lcd.print(F("Locked  "));
  }
  delay(1500);

  timerElapsed = 0;
}

//------------------------------------------------------------------------------------------------------------------------
void processTransmitter() {
  senseVoltage(VOLTAGE_SENSE_PIN);
  maintainTimer(channelValues[THROTTLE_CHANNEL], pgm_read_word(&channelLowOut [currentModel][THROTTLE_CHANNEL]), pgm_read_word(&timeWarning [currentModel]), pgm_read_word(&timeAlert [currentModel]));
  bool dataChanged = getChannelValues();    // Get channel data and note if any value changes
  checkButtons();  // do this before applymixes so trim saves are based on pre-mixed stick reads that were just obtained

//  for(int x = 0; x < 2 ; x++) {
//    Serial.print(channelValues[x]); Serial.print(F("\t")); 
//  } 

  applyMixes();
  sendPacket();

}


//------------------------------------------------------------------------------------------------------------------------
void timerService() {

  if (useV3Xmit) {
    sendPacket_V3_XMIT();
  } 
  else if (PPMEnabled()) {
     interrupts();          // re-enable interrupts so that PPM timing is not messed up
  }
  
  // These dont need to be called as frequently as every interupt, so only call half of them each time through
  static bool evenPass = false;
  if (evenPass) {
    rateButton.service();
    timerResetButton.service();
    modelSelectButton.service();  
    trimButton.service();
    buzzerService();
  }
  else
  {    
    int throwAwayRead = analogRead(ANALOG_TRIM_BUTTON_PIN);  //Read once to allow MUX to settle so first button read is more accurate.
    // All the trim buttons are on the same analog pin so no need for throw away read on each call.
    trimPitchUpButton.service();
    trimPitchDownButton.service();
    trimRollLeftButton.service();
    trimRollRightButton.service();
    trimYawLeftButton.service();
    trimYawRightButton.service();
  }
  evenPass = !evenPass;
}

//------------------------------------------------------------------------------------------------------------------------
void buzzerService() {

  static bool buzzerPinState = false;  
  static unsigned long lastPinTransition = 0;

  switch(buzzerState)
  {
    case OFF  : 
                if (buzzerPinState == true) {
                  digitalWrite(BUZZER_PIN, LOW);
                  buzzerPinState = false;   
                  lastPinTransition = millis();
                }
                break;
    case ON   : 
                if (buzzerPinState == false) {
                  digitalWrite(BUZZER_PIN, HIGH);
                  buzzerPinState = true;   
                  lastPinTransition = millis();
                }
                break;
    case PULSE_FAST : 
    case PULSE_SLOW : 
                unsigned long currentTime = millis();
                int interval = (buzzerState == PULSE_FAST) ? 2000 : 12000;
                if (buzzerPinState && ((currentTime - lastPinTransition) > 100) || (!buzzerPinState && (currentTime - lastPinTransition) > interval)) {
                  buzzerPinState = !buzzerPinState;   
                  digitalWrite(BUZZER_PIN, buzzerPinState);
                  lastPinTransition = currentTime;                  
                }                
                break;
  }      
}

//------------------------------------------------------------------------------------------------------------------------
void checkButtons() {
  ClickEncoder::Button trimButtonState        = trimButton.getButton();
  ClickEncoder::Button modelSelectButtonState =  modelSelectButton.getButton();
  ClickEncoder::Button timerResetButtonState  =  timerResetButton.getButton();
  ClickEncoder::Button rateButtonState        =  rateButton.getButton();
  
  ClickEncoder::Button trimPitchUpButtonState        =  trimPitchUpButton.getButton();
  ClickEncoder::Button trimPitchDownButtonState        =  trimPitchDownButton.getButton();
  ClickEncoder::Button trimRollLeftButtonState        =  trimRollLeftButton.getButton();
  ClickEncoder::Button trimRollRightButtonState        =  trimRollRightButton.getButton();
  ClickEncoder::Button trimYawLeftButtonState        =  trimYawLeftButton.getButton();
  ClickEncoder::Button trimYawRightButtonState        =  trimYawRightButton.getButton();

  if (trimPitchDownButtonState == ClickEncoder::Clicked) {
    modelData.trimData[PITCH_CHANNEL] = ((bool)(pgm_read_byte( &channelReverse[currentModel][PITCH_CHANNEL]))) ? modelData.trimData[PITCH_CHANNEL] + TRIM_BUTTON_STEP_SIZE : modelData.trimData[PITCH_CHANNEL] - TRIM_BUTTON_STEP_SIZE ;
    Serial.println(F("PitchDown"));
  }

  if (trimPitchUpButtonState == ClickEncoder::Clicked) {
    modelData.trimData[PITCH_CHANNEL] = ((bool)(pgm_read_byte( &channelReverse[currentModel][PITCH_CHANNEL]))) ? modelData.trimData[PITCH_CHANNEL] - TRIM_BUTTON_STEP_SIZE : modelData.trimData[PITCH_CHANNEL] + TRIM_BUTTON_STEP_SIZE;
    Serial.println(F("PitchUp"));
  }
  if (trimRollLeftButtonState == ClickEncoder::Clicked) {
    modelData.trimData[ROLL_CHANNEL] = ((bool)(pgm_read_byte( &channelReverse[currentModel][ROLL_CHANNEL]))) ? modelData.trimData[ROLL_CHANNEL] + TRIM_BUTTON_STEP_SIZE  : modelData.trimData[ROLL_CHANNEL] - TRIM_BUTTON_STEP_SIZE;
    Serial.println(F("RollLeft"));
  }

  if (trimRollRightButtonState == ClickEncoder::Clicked) {
    modelData.trimData[ROLL_CHANNEL] = ((bool)(pgm_read_byte( &channelReverse[currentModel][ROLL_CHANNEL]))) ? modelData.trimData[ROLL_CHANNEL] - TRIM_BUTTON_STEP_SIZE : modelData.trimData[ROLL_CHANNEL] + TRIM_BUTTON_STEP_SIZE ;
    Serial.println(F("RollRight"));
  }

  if (trimYawLeftButtonState == ClickEncoder::Clicked) {
    modelData.trimData[YAW_CHANNEL] = ((bool)(pgm_read_byte( &channelReverse[currentModel][YAW_CHANNEL]))) ? modelData.trimData[YAW_CHANNEL] - TRIM_BUTTON_STEP_SIZE  : modelData.trimData[YAW_CHANNEL] + TRIM_BUTTON_STEP_SIZE;
    Serial.println(F("YawLeft"));
  }

  if (trimYawRightButtonState == ClickEncoder::Clicked) {
    modelData.trimData[YAW_CHANNEL] = ((bool)(pgm_read_byte( &channelReverse[currentModel][YAW_CHANNEL]))) ? modelData.trimData[YAW_CHANNEL] + TRIM_BUTTON_STEP_SIZE : modelData.trimData[YAW_CHANNEL] - TRIM_BUTTON_STEP_SIZE ;
    Serial.println(F("YawRight"));
  }

  if (rateButtonState == ClickEncoder::Clicked) {
    highRate = !highRate;
    displayRateSetting();
  }
  
  if ((trimButtonState == ClickEncoder::Released) && !setFailSafeValues)      setTrim();
  if (trimButtonState == ClickEncoder::DoubleClicked) reSetTrim();

  if (timerResetButtonState == ClickEncoder::DoubleClicked) timerElapsed = 0;

  if (!modelLocked) {
    if (modelSelectButtonState == ClickEncoder::Clicked)     setCurrentModel(currentModel+1);
  }
  if (!bindMode && !modelLocked && (modelSelectButtonState == ClickEncoder::Released)) {
    modelLocked = true;              // Extended hold locks ability to change the model
    Serial.println(F("Model Locked"));
    lcd.setCursor(0,0);;lcd.print(F("Locked  "));
  }
  if (!bindMode && modelLocked && (modelSelectButtonState == ClickEncoder::DoubleClicked)) {
    modelLocked = false;             // Double click unlocks ability to change the model
    Serial.println(F("Model Unlocked"));
    lcd.setCursor(0,0);;lcd.print(F("Unlcked "));
  }
  if ((timerResetButtonState == ClickEncoder::Held) && (rateButtonState == ClickEncoder::Held)) {
    setFailSafeValues = true;
  } else {
    setFailSafeValues = false;
  }
}

//------------------------------------------------------------------------------------------------------------------------
void reSetTrim() { 

  for (int x = 0; x < NUM_CHANNELS; x++) {
    modelData.trimData[x] = 0;
  }
  EEPROM.put(modelDataEEPROMAddress,modelData);
  Serial.println(F("Trim Reset"));
  
}

//------------------------------------------------------------------------------------------------------------------------
void setTrim() {  
  modelData.trimData[ROLL_CHANNEL]  = (int16_t)channelValues[ROLL_CHANNEL]  - CHANNEL_MID_VALUE;
  modelData.trimData[PITCH_CHANNEL] = (int16_t)channelValues[PITCH_CHANNEL] - CHANNEL_MID_VALUE;
  modelData.trimData[YAW_CHANNEL]   = (int16_t)channelValues[YAW_CHANNEL]   - CHANNEL_MID_VALUE;

  EEPROM.put(modelDataEEPROMAddress,modelData);
  Serial.println(F("Trimmed"));
}


//------------------------------------------------------------------------------------------------------------------------
void setCurrentModel(uint8_t newModel) {  
  currentModel = newModel;
  if (currentModel >= NUM_MODELS) currentModel = 0;   // wrap around if we try to exceed max models
  EEPROM.put(currentModelEEPROMAddress,currentModel);
  EEPROM.get(modelDataEEPROMAddress,modelData);       // load the trim data
  Serial.print(F("Current Model Number: "));Serial.println(currentModel);
  modelNameType currentModelName;
  memcpy_P (&currentModelName, &modelNames [currentModel], sizeof currentModelName);
  Serial.print(F("Current Model Name: "));Serial.println(currentModelName.modelName);
  lcd.setCursor(0,1);lcd.print(LCD_blankLine);
  lcd.setCursor(0,1);lcd.print(currentModelName.modelName);
  highRate = 0;    // Set to low rates when model changes
  displayRateSetting();
  if (!bindMode) {
    radioPipeID = radioNormalTxPipeID  + (((uint64_t)currentModel)<<8) + ((uint64_t)currentModel);
    changeWritingPipe(radioPipeID);   // Change pipe id based on model num to be compatible with Multi Module behavior
    getChannelSequence (radioChannel, CABELL_RADIO_CHANNELS, radioPipeID); 
  }
  Serial.print(F("Radio ID: "));Serial.print((uint32_t)(radioPipeID>>32)); Serial.print(F("\t"));Serial.println((uint32_t)((radioPipeID<<32)>>32));
}

//------------------------------------------------------------------------------------------------------------------------
void displayRateSetting() {
  lcd.setCursor(14,1);
  if (highRate) {
    lcd.print("Hi");
  } else {
    lcd.print("Lo");
  }
  
}

//------------------------------------------------------------------------------------------------------------------------
void changeWritingPipe(uint64_t newRadioPipeID) {
    radio.flush_tx();

    // Need to cycle thorugh reading mode to get a change in radioPipeID to work properly
    radio.openReadingPipe(0,newRadioPipeID);
    radio.startListening();
    radio.stopListening();
    radio.closeReadingPipe(newRadioPipeID);
       
    radio.openWritingPipe(newRadioPipeID);  
}

//------------------------------------------------------------------------------------------------------------------------
void sendPacket() {
  static uint32_t prevReciverVer = 0;
  bool firstReciverVerPacket = true;
  uint32_t reciverVer = pgm_read_dword(&recieverVersion [currentModel]);

  if (reciverVer != prevReciverVer) {
    firstReciverVerPacket = true;
    radio.disableDynamicPayloads();
    if (PPMEnabled()) ppmDisable();
    radio.setPayloadSize(32);        //set to default (max) size, which is used for V1 and V2
    useV3Xmit        = false;
    changeWritingPipe(radioPipeID);  // Reset the TX ID in case it was changed below.  radioPipeID contains either normal or bind value depending how TX was booted
  }
  else
  {
    firstReciverVerPacket = false;    
  }
  
  if (reciverVer == 0) {
    sendPacket_PPM();        // first so we have something to send  when PPM enabled
    if (!PPMEnabled()) {
      ppmSetup(PPM_OUTPUT_PIN, min(NUM_CHANNELS,pgm_read_byte(&channelsToXmit[currentModel])));
    }
  }

  if (reciverVer == 3) {
    if (firstReciverVerPacket) {
      radio.enableDynamicPayloads();
      sendPacket_V3();  
      useV3Xmit = true;  // last so we are sure to have a valid packet to xmit before interrupt routine tries to send something
    }
    else {
      sendPacket_V3();  
    }
  }

  if (((reciverVer>>24)&0xFE) == 0x54) sendPacket_Serial(reciverVer);    // Serial for the Multiprotocol module if first byte is 0x54 or 0x55

  prevReciverVer =reciverVer;
}

//------------------------------------------------------------------------------------------------------------------------
void sendPacket_V3() {

  CABELL_RxTxPacket_t TxPacket;
  
  uint8_t channelsToSend = constrain( pgm_read_byte(&channelsToXmit[currentModel]),CABELL_MIN_CHANNELS,CABELL_NUM_CHANNELS);
  if (bindMode) 
    channelsToSend = CABELL_NUM_CHANNELS;
    
  uint8_t channelReduction =  constrain(((CABELL_NUM_CHANNELS - channelsToSend)),0,CABELL_NUM_CHANNELS - CABELL_MIN_CHANNELS);    // min is 4 channels
  uint8_t packetSize = sizeof(TxPacket) - ((((channelReduction - (channelReduction%2))/ 2)) * 3);      // reduce 3 bytes per 2 channels, but not last channel if it is odd
  uint8_t maxPayloadValueIndex = sizeof(TxPacket.payloadValue) - (sizeof(TxPacket) - packetSize);

  TxPacket.RxMode = (bindMode) ?  CABELL_RxTxPacket_t::RxMode_t::bind : ((setFailSafeValues) ? CABELL_RxTxPacket_t::RxMode_t::setFailSafe : CABELL_RxTxPacket_t::RxMode_t::normal);
  if (telemetryEnabled && (TxPacket.RxMode == CABELL_RxTxPacket_t::RxMode_t::normal)) {
    TxPacket.RxMode = CABELL_RxTxPacket_t::RxMode_t::normalWithTelemetry;
  }
  TxPacket.option = channelReduction & CABELL_OPTION_MASK_CHANNEL_REDUCTION; 
  TxPacket.option |= ( pgm_read_byte(&recieverOutputMode[currentModel]) << CABELL_OPTION_SHIFT_RECIEVER_OUTPUT_MODE) & CABELL_OPTION_MASK_RECIEVER_OUTPUT_MODE;
  TxPacket.reserved = 0;
  TxPacket.modelNum = currentModel;
  uint16_t checkSum = TxPacket.modelNum + TxPacket.option + TxPacket.RxMode  + TxPacket.reserved;    // Start Calculate checksum

//  Serial.print(F("sizeof(TxPacket)     "));Serial.println(sizeof(TxPacket));
//  Serial.print(F("channelsToSend       "));Serial.println(channelsToSend);
//  Serial.print(F("channelReduction     "));Serial.println(channelReduction);
//  Serial.print(F("packetSize           "));Serial.println(packetSize);
//  Serial.print(F("maxPayloadValueIndex "));Serial.println(maxPayloadValueIndex);
  
  int payloadIndex = 0;
  uint16_t holdValue;
  
  for (int adjusted_x = 0;(adjusted_x < channelsToSend); adjusted_x++) {
    holdValue = (adjusted_x < NUM_CHANNELS) ? (channelValues[adjusted_x]) : CHANNEL_MIN_VALUE;     // valid channel values are 1000 to 2000
//    Serial.print(holdValue); Serial.print(F("\t")); 
    if (bindMode) {
      switch (adjusted_x) {
        case THROTTLE_CHANNEL : holdValue = CHANNEL_MIN_VALUE; break;      // always set throttle to zero when binding for safety
        case 11       : holdValue = 1000 + ((((uint64_t)radioNormalTxPipeID)>>32) & 0x00000000000000FF); break;
        case 12       : holdValue = 1000 + ((((uint64_t)radioNormalTxPipeID)>>24) & 0x00000000000000FF); break;
        case 13       : holdValue = 1000 + ((((uint64_t)radioNormalTxPipeID)>>16) & 0x00000000000000FF); break;
        case 14       : holdValue = 1000 + ((((uint64_t)radioNormalTxPipeID)>>8)  & 0x00000000000000FF); break;
        case 15       : holdValue = 1000 + ((((uint64_t)radioNormalTxPipeID))     & 0x00000000000000FF); break;
      }
    }

    // use 12 bits per value
    if (adjusted_x % 2) {     //channel number is ODD
      holdValue = holdValue<<4; 
      payloadIndex--;     
    } else {
      holdValue &= 0x0FFF;
    }
    TxPacket.payloadValue[payloadIndex] |=  (uint8_t)(holdValue & 0x00FF);
    payloadIndex++;
    TxPacket.payloadValue[payloadIndex] |=  (uint8_t)((holdValue>>8) & 0x00FF);   
    payloadIndex++;
  }
  
  for(int x = 0; x < maxPayloadValueIndex ; x++) {
    checkSum += TxPacket.payloadValue[x];  // Finish Calculate checksum 
//    Serial.print(TxPacket.payloadValue[x],HEX); Serial.print(F("\t")); 
  } 

  TxPacket.checkSum_MSB = checkSum >> 8;
  TxPacket.checkSum_LSB = checkSum & 0x00FF;
  
//  Serial.print(TxPacket.checkSum); Serial.print(F("\t")); 
//  Serial.print(millis());; Serial.print(F("\t"));    
  
  V3PacketSize = packetSize;
  memcpy (&V3SendBuffer[0], &TxPacket, packetSize);
  newV3PacketReady = true;

}

//------------------------------------------------------------------------------------------------------------------------
void sendPacket_V3_XMIT() {
  
  static int currentChannel = CABELL_RADIO_MIN_CHANNEL_NUM;  // Initializes the channel sequence. 
  
  currentChannel = getNextChannel (radioChannel, CABELL_RADIO_CHANNELS, currentChannel);
  radio.setChannel(currentChannel);

  static uint8_t xmitPacketSize = V3PacketSize;
  static uint8_t xmitPacketBuf[32];
  
  if (newV3PacketReady) {           // Only bring in new data to xmit when flagged ready.  This is to ensure we dont take a packet in the process of being built.
    xmitPacketSize = V3PacketSize;
    memcpy (&xmitPacketBuf[0], &V3SendBuffer[0], V3PacketSize);
  }
  newV3PacketReady = false;

  #define RESERVED_POSITION 1
  #define CHECKSUM_LSB_POSITION 4
  #define CHECKSUM_MSB_POSITION 5

  uint16_t checkSum = xmitPacketBuf[CHECKSUM_MSB_POSITION] << 8;
  checkSum += xmitPacketBuf[CHECKSUM_LSB_POSITION];
  checkSum -= xmitPacketBuf[RESERVED_POSITION];
  checkSum += currentChannel;
  xmitPacketBuf[RESERVED_POSITION] = currentChannel;
  xmitPacketBuf[CHECKSUM_MSB_POSITION] = checkSum >> 8;
  xmitPacketBuf[CHECKSUM_LSB_POSITION] = checkSum & 0x00FF;
  
  radio.setPayloadSize(xmitPacketSize);  

//  static unsigned long int lastSendMicros = 0;
//  static unsigned long int exited;
//  lastSendMicros = micros();

//    static int8_t packetCounter = 0;  
//    packetCounter++;
//    xmitPacketBuf[0] &= 0x7F;               // clear 8th bit
//    xmitPacketBuf[0] |= packetCounter<<7;   // This causes the 8th bit of the first byte to toggle with each xmit so consecrutive payloads are not identical.  This is a work around for a reported bug in clone NRF24L01 chips that mis-took this case for a re-transmit of the same packet.
    
//  if (packetCounter == 0)
   radio.startFastWrite( &xmitPacketBuf[0], xmitPacketSize,0); 

//  exited = micros();
//  Serial.print(lastSendMicros); Serial.print(F("\t")); Serial.println(exited); Serial.print(F("\t")); Serial.println(packetSize);    
}

//------------------------------------------------------------------------------------------------------------------------
void sendPacket_PPM() {   // output as AETR
  int adjusted_x;
  for(int x = 0; x < constrain( pgm_read_byte(&channelsToXmit[currentModel]),CABELL_MIN_CHANNELS,8) ; x++) {  //  sending more than 8 on PPM may be too slow 

    //set adjusted_x to be in AETR order
    switch (x)
    {
      case 0:
        adjusted_x = ROLL_CHANNEL;
        break;
      case 1:
        adjusted_x = PITCH_CHANNEL;
        break;
      case 2:
        adjusted_x = THROTTLE_CHANNEL;
        break;
      case 3:
        adjusted_x = YAW_CHANNEL;
        break;
      default:
        adjusted_x = x;
    }
    
    setPPMOutputChannelValue(x, channelValues[adjusted_x]);
  //  Serial.print(channelValues[x]); Serial.print(F("\t")); 
  } 
  //Serial.println(millis()); 
}

//------------------------------------------------------------------------------------------------------------------------
void sendPacket_Serial(uint32_t recieverProtocol){

#define SERIAL_OUTPUT_BUFFER_LENGTH 26

/* 
The cabell sub-protocol number is 63
0) 0x54  add 32 to sub protocol
1) 0x1F  32 + 21 = sub protocol 63, Autobind is 0 (off)
2) 0x00  Type is 0 as there are no types in cabell sub-protocol
3) 0x00  No options
*/
  uint8_t outputBuf[SERIAL_OUTPUT_BUFFER_LENGTH] = {0};      // initialize array to zeroes
  static uint8_t bindModeResetCount = 0;
  
  //recieverProtocol is first 4 bytes of serial packet, with sole forced overrides
  outputBuf[0]  = (uint8_t)((recieverProtocol & 0xFF000000) >> 24);     // first byte is 0x55 for sub-protocols 0 - 31, 0x54 for 32 - 63 

  outputBuf[1]  = (uint8_t)((recieverProtocol & 0x001F0000) >> 16);    //sub-protocol is first 5 bits of second byte
  outputBuf[1] |= 0x00 & 0x20;                                         //Range check is sixth bit  - force off
  outputBuf[1] |= (uint8_t)((recieverProtocol & 0x00400000) >> 16);    //Autobind is seventh bit - copy from recieverProtocol
  
  if (!(recieverProtocol & 0x00400000)) {  // if Autobind is not on, then set the bind bit if TX in bind mode
    // occasionally leave the bit set to zero - this toggling re-sets the protocol in the module, thus keeping it in bind mode
    if (bindModeResetCount != 0) {     // trigger the toggle every 255 packets
      outputBuf[1] |= (uint8_t)((bindMode) ? 0x80 : 0x00);    //Bind is eighth bit - set if TX is in bind mode
    }
    bindModeResetCount++;
  }
  outputBuf[2]  = 0x00;                                             // last bit is power.  Set to 0 which is high power.  Other bits filled in next
  outputBuf[2] |= ((uint8_t)currentModel) & 0x0F;                   //Model number 0 to 15 into first 4 bits.  If currentModel > 16 it wraps around
  outputBuf[2] |= (uint8_t)((recieverProtocol & 0x00007000) >>8);   //Type is bits 4,5,6 of third byte- copy from RecieverProtocol
  
  outputBuf[3]  = (uint8_t)(recieverProtocol & 0x00000040);   // 7th bit is used to turn on max power. use bit from recieverProtocol
  outputBuf[3] |= (constrain(((CABELL_NUM_CHANNELS - pgm_read_byte(&channelsToXmit[currentModel]))),0,CABELL_NUM_CHANNELS - CABELL_MIN_CHANNELS)) & CABELL_OPTION_MASK_CHANNEL_REDUCTION;
  outputBuf[3] |= (pgm_read_byte(&recieverOutputMode[currentModel]) << CABELL_OPTION_SHIFT_RECIEVER_OUTPUT_MODE) & CABELL_OPTION_MASK_RECIEVER_OUTPUT_MODE;
  
  // Set the model number

  uint8_t model = ((uint8_t)currentModel) | 0x0F;   // first 4 bits is model number
  uint8_t Power = ~0x80;   // last bit is power.  0 is high power
  
  int currByte = 4;  // start values after the 4 byte header
  int currBit = 0;   // start wit the first bit
  int adjusted_x;
  for (int x = 0; x < min(NUM_CHANNELS,16); x++) {    //Serial protocol max is 16 channels
    switch (x)
    {
      case 0:
        adjusted_x = ROLL_CHANNEL;
        break;
      case 1:
        adjusted_x = PITCH_CHANNEL;
        break;
      case 2:
        adjusted_x = THROTTLE_CHANNEL;
        break;
      case 3:
        adjusted_x = YAW_CHANNEL;
        break;
      default:
        adjusted_x = x;
    }
    uint32_t chanVal = map(channelValues[adjusted_x],CHANNEL_MIN_VALUE,CHANNEL_MAX_VALUE,204,1843);
//    Serial.print(chanVal);Serial.print(F("\t"));
    // use 11 bits per value. do funky stuff because little endian
    uint32_t shiftedValue = chanVal<<currBit;
    uint8_t firstbyte =( shiftedValue & 0x000000FF);
    uint8_t Secondbyte =( shiftedValue & 0x0000FF00)>>8;
    uint8_t Thirdbyte =( shiftedValue & 0x00FF0000)>>16;
    outputBuf[currByte] |= firstbyte;
    currByte++;
    outputBuf[currByte] |= Secondbyte;
    if (currBit > 5) {
      currByte++;
      outputBuf[currByte] |= Thirdbyte;      
    }
    currBit = (currBit + 11) % 8;
  }
  

//  for (int x = 0; x < SERIAL_OUTPUT_BUFFER_LENGTH; x++) {    
//    Serial.print(outputBuf[x],HEX);Serial.print(F("\t"));
//  }
//  Serial.println(micros());

  static long unsigned int lastSerialPacket = 0;
  if (lastSerialPacket > micros()) lastSerialPacket = 0; // reset if micros overflows (approx every 70 min)

  if (micros() > (lastSerialPacket + 7000)) {     // make sure we dont send any more than every 7000 uS so module doesnt get confused
    lastSerialPacket = micros();    
    MsTimer2::stop();   // Stop timer while sending so timing isnt interrupted
    outputSerial.writeBuffer32(outputBuf,SERIAL_OUTPUT_BUFFER_LENGTH);
    MsTimer2::start();     
  }
}

//------------------------------------------------------------------------------------------------------------------------
bool getChannelValues () {
  bool dataChanged = false;
  channelValues[PITCH_CHANNEL]    = getChannelValueFromADC(dataChanged,channelValues[PITCH_CHANNEL],ads,PITCH_ADC_PIN,PITCH_CHANNEL);
  channelValues[ROLL_CHANNEL]     = getChannelValueFromADC(dataChanged,channelValues[ROLL_CHANNEL],ads,ROLL_ADC_PIN,ROLL_CHANNEL);
  channelValues[YAW_CHANNEL]      = getChannelValueFromADC(dataChanged,channelValues[YAW_CHANNEL],ads,YAW_ADC_PIN,YAW_CHANNEL);
  channelValues[THROTTLE_CHANNEL] = getChannelValueFromADC(dataChanged,channelValues[THROTTLE_CHANNEL],ads,THROTTLE_ADC_PIN,THROTTLE_CHANNEL);
  channelValues[AUX1_CHANNEL]     = getChannelValueFromPin(dataChanged,channelValues[AUX1_CHANNEL],AUX1_INPUT_PIN,AUX1_CHANNEL);
  channelValues[AUX2_CHANNEL]     = getChannelValueFromPin(dataChanged,channelValues[AUX2_CHANNEL],AUX2_INPUT_PIN,AUX2_CHANNEL);
  channelValues[AUX3_CHANNEL]     = getChannelValueFromPin(dataChanged,channelValues[AUX3_CHANNEL],AUX3_INPUT_PIN,AUX3_CHANNEL);
  channelValues[AUX4_CHANNEL]     = getChannelValueFromPin(dataChanged,channelValues[AUX4_CHANNEL],AUX4_INPUT_PIN,AUX4_CHANNEL);

  for(int x = AUX4_CHANNEL + 1; x < CABELL_NUM_CHANNELS; x++) {
    channelValues[x] = channelValues[x-4];                        // Repeat AUX channels to fill in upper channels
  }
  return dataChanged;
}

//------------------------------------------------------------------------------------------------------------------------
uint16_t getChannelValueFromPin(bool& changed, uint16_t originalValue, uint8_t pin, int channel) {
  uint16_t x;
  if (pgm_read_byte( &channelReverse[currentModel][channel])) {
    x = (digitalRead(pin) == HIGH) ?  pgm_read_word(&channelLowOut [currentModel][channel]) - (highRate * pgm_read_word(&channelHighRateOffset [currentModel][channel])) :  pgm_read_word(&channelHighOut [currentModel][channel] + (highRate * pgm_read_word(&channelHighRateOffset [currentModel][channel])));
  } else {
    x = (digitalRead(pin) == HIGH) ?  pgm_read_word(&channelHighOut [currentModel][channel]) + (highRate * pgm_read_word(&channelHighRateOffset [currentModel][channel])) : pgm_read_word(&channelLowOut [currentModel][channel] - (highRate * pgm_read_word(&channelHighRateOffset [currentModel][channel])));
  }
  x = constrain(x,CHANNEL_MIN_VALUE,CHANNEL_MAX_VALUE);
  if (x != originalValue) {changed = true;}
  return x;
}

//------------------------------------------------------------------------------------------------------------------------
uint16_t getChannelValueFromADC(bool& changed, uint16_t originalValue, ADC_VERSION adc, uint8_t adcPin, int channel) {
  static const int16_t lowInput[4]     = ADC_INPUT_LOW ;
  static const int16_t midInput[4]     = ADC_INPUT_MID ;
  static const int16_t highInput[4]    = ADC_INPUT_HIGH ;
  
  int16_t x = adc.readADC_SingleEnded(adcPin);
//   Serial.print(x);Serial.print(F("\t"));
  x = constrain(x,lowInput[channel],highInput[channel]);

  // calculate mid points on either side of the dead zone
  int16_t midInputLow = midInput[channel] - (int16_t) (((float) midInput[channel] - (float) lowInput[channel]) * STICK_DEAD_ZONE_RATIO);
  int16_t midInputHigh = midInput[channel] + (int16_t) (((float) highInput[channel] - (float) midInput[channel]) * STICK_DEAD_ZONE_RATIO);;

  if (x < midInputLow) {
    x = map(x,lowInput[channel],midInputLow,CHANNEL_MIN_VALUE,CHANNEL_MID_VALUE);
  } else if (x > midInputHigh) {
    x = map(x,midInputHigh,highInput[channel],CHANNEL_MID_VALUE,CHANNEL_MAX_VALUE);
  } else {
    x = CHANNEL_MID_VALUE;
  }

  uint16_t xOut = mapChannelValue(channel,x);      // maps based on channel min/max settings
       
  if (xOut != originalValue) {changed = true;}
  return xOut;
}

//------------------------------------------------------------------------------------------------------------------------
uint16_t applyExpo (int8_t inExpoPercent, uint16_t inValue, uint16_t inLow, uint16_t inMid, uint16_t inHigh) {

  float expoValue = (int16_t) inValue - (int16_t) inMid;
 
  float expoMod;
  if (expoValue < 0) {
    expoMod = ((float) (100 - inExpoPercent) + ((float) inExpoPercent) / ((float)inMid  - (float)(inLow)) * abs(expoValue)) / 100.0F ;
  } else {
    expoMod = ((float) (100 - inExpoPercent) + ((float) inExpoPercent) / ((float)inHigh - (float)(inMid)) * abs(expoValue)) / 100.0F ;   
  }
  expoValue = expoValue * expoMod;
  
  return (uint16_t) (expoValue + inMid);
}

//------------------------------------------------------------------------------------------------------------------------
void maintainTimer(uint16_t inValue, uint16_t inLow, int timerWarningSec, int timerAlertSec) {

  static unsigned long lastMillis = 0;
  static unsigned long lastSeconds = 0;
  unsigned long newMillis =  millis();

  if (newMillis < lastMillis) {     //  Handle when millis() wraps 
    lastMillis = 0;
  } 

  if(inValue > inLow) {       // only increment timer if throttle is not min
    timerElapsed = timerElapsed + (newMillis - lastMillis);
  } 

  static char displayedTime[12] = "";
  char * newTime;
  if ((timerElapsed/1000) != lastSeconds) {
    lastSeconds = timerElapsed/1000;
    newTime = TimeToString(lastSeconds);
    // writing string to lcd via I2C is slow, so compate to last string to find the first different characdter and 
    // only write from that point forward.  This saves about 6ms.
    int stringDiffIndex = findFirstStringDifference(newTime,displayedTime,sizeof(displayedTime));
    lcd.setCursor(8 + stringDiffIndex, 0);
    for (int x = stringDiffIndex; x < 8; x++) {
      lcd.print(newTime[x]);
    }
    copyString(newTime, displayedTime, sizeof(displayedTime));
  }

  static bool alertOrWarning = false;

  if ((lastSeconds >= timerAlertSec) && (timerAlertSec > 0)) {
    buzzerState=PULSE_FAST;
    alertOrWarning = true;
  } else {
    if ((lastSeconds >= timerWarningSec) && (timerWarningSec > 0)) {
      buzzerState=PULSE_SLOW;
      alertOrWarning = true;
    } else {
      if (alertOrWarning) {
        buzzerState=OFF;
        alertOrWarning = false;
      }
    }
  }
    
  lastMillis = newMillis;
}

//------------------------------------------------------------------------------------------------------------------------
int findFirstStringDifference(char * first, char * second, int arrayLength) {
  int x;
  for (x = 0; x < arrayLength; x++) {
    if (first[x] != second[x]) break;
  }
  return x;
}


//------------------------------------------------------------------------------------------------------------------------
void copyString(char * first, char * second, int arrayLength) {
  for (int x = 0; x < arrayLength; x++) {
    second[x] = first[x];
  }
}

//------------------------------------------------------------------------------------------------------------------------
// t is time in seconds;
char * TimeToString(unsigned long t)
{
  static char str[12];
  long h = t / 3600;
  t = t % 3600;
  int m = t / 60;
  int s = t % 60;
  if (h > 99) {
    h = 99;
    m = 99;
    s = 99;
  }
  sprintf(str, "%02ld:%02d:%02d", h, m, s);
  return str;
}

//------------------------------------------------------------------------------------------------------------------------
void senseVoltage(int sensePin) {
  
  // Only sense the battery voltage once every CHECK_COUNT times.  This reduces overhead of 
  // doing the analog read in each loop and reduces how often the analog MUX has to reset
  // Also allows the LCD to display alternate message breifly before retuning to the low battery warning
  #define CHECK_COUNT 700

  static int callCount = 0;

  if ( callCount++ >= CHECK_COUNT) {
    callCount = 0;
    int voltageReading = analogRead(sensePin);  // Priming read to set the analog mux so second read is more accurate
    voltageReading = analogRead(sensePin);    
    if (LOW_VOLTAGE_THRESHOLD >= voltageReading) {
      lcd.setCursor(0,1);;lcd.print(F("  Battery Low!  "));
    }    
  }
}

//------------------------------------------------------------------------------------------------------------------------
void applyMixes() {
  #define NO_MIX_DEFINED 32000
  int16_t mixedValues [NUM_CHANNELS]; 
  for (int x = 0; x < NUM_CHANNELS; x++) {
    mixedValues[x] = NO_MIX_DEFINED;
//    Serial.print(channelValues[x]);Serial.print(F("\t"));
  }
//  Serial.println();

  for (int y = 0; (y < MODEL_MAX_MIXES) && ((int8_t pgm_read_byte(&mixItem[currentModel][y].destChannel)) != LAST_MIX_ENTRY); y++) {
    int8_t destChannel   = pgm_read_byte(&mixItem[currentModel][y].destChannel);
    int8_t sourceChannel = pgm_read_byte(&mixItem[currentModel][y].sourceChannel);
    int8_t mixPercent    = pgm_read_byte(&mixItem[currentModel][y].mixPercent);
    if ((destChannel >= 0) && (destChannel < NUM_CHANNELS) && (sourceChannel >= 0) && (sourceChannel < NUM_CHANNELS)) {  //Ignore mixes for bad channels
      if (mixedValues[destChannel] == NO_MIX_DEFINED) mixedValues[destChannel] = 0;
      mixedValues[destChannel] = ((float) mixedValues[destChannel]) + ((((float) channelValues[sourceChannel]) - ((float) CHANNEL_MID_VALUE)) * (((float) mixPercent/100.0)));
    }
  }  
  for (int x = 0; x < NUM_CHANNELS; x++) {
    if (mixedValues[x] != NO_MIX_DEFINED) {
      channelValues[x] = constrain(mixedValues[x] + CHANNEL_MID_VALUE,uint16_t pgm_read_word(&channelLowOut [currentModel][x]),uint16_t pgm_read_word(&channelHighOut [currentModel][x]));
   }
//    Serial.print(channelValues[x]);Serial.print(F("\t"));
  }
//  Serial.println();
//  Serial.println();
}

ISR(TIMER1_COMPA_vect){
  SUM_PPM_ISR();
}



  
