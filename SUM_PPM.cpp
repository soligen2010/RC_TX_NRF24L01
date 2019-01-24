
#include "Config.h"

volatile uint8_t ppmPin = 0xFF;  // initialized to invalid pin
volatile int16_t ppmValueArray [CABELL_NUM_CHANNELS]; 
volatile uint8_t ppmChannelCount; 
bool ppmEnabled = false;


//------------------------------------------------------------------------------------------------------------------------
void ppmSetup(uint8_t pin, uint8_t channelCount){  
  //this program will put out a PPM signal

  // from: https://code.google.com/archive/p/generate-ppm-signal/
  
  //////////////////////CONFIGURATION///////////////////////////////
  #define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
  #define PPM_MaxChannels 8  //The maximum number of channels that can be sent in a frame
  #define PPM_PulseLen 300  //set the pulse length
  #define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
  //////////////////////////////////////////////////////////////////

  ppmPin = pin;
  ppmChannelCount = min(PPM_MaxChannels,channelCount);
  
  pinMode(ppmPin, OUTPUT);
  digitalWrite(ppmPin, !onState);  //set the PPM signal pin to the default state (off)

  noInterrupts();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  interrupts();
  ppmEnabled = true;

}

bool PPMEnabled() {
  return ppmEnabled;
}

//------------------------------------------------------------------------------------------------------------------------
void ppmDisable(){  
  
  noInterrupts();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  TIMSK1 &= ~(1<<OCIE1A);   // Disable Interrupt Counter 1, output compare A (TIMER1_CMPA_vect)
  interrupts();
  ppmEnabled = false;

}

//------------------------------------------------------------------------------------------------------------------------
void setPPMOutputChannelValue(uint8_t channel, int value) {
  ppmValueArray[channel] = value;
}

//------------------------------------------------------------------------------------------------------------------------
void SUM_PPM_ISR() {  
  static boolean state = true;
  static byte cur_chan_numb = 0;
  static unsigned int calc_rest = 0;
  
  if(state) {  //start pulse
    digitalWrite(ppmPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
  
    digitalWrite(ppmPin, !onState);
    state = true;

    if(cur_chan_numb >= ppmChannelCount){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      int16_t ppmValue = constrain(ppmValueArray[cur_chan_numb],CHANNEL_MIN_VALUE,CHANNEL_MAX_VALUE);
      OCR1A = (ppmValue - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppmValue;
      cur_chan_numb++;
    }     
  }
}

