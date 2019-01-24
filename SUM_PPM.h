
#ifndef __have__SUM_PPM_h__
#define __have__SUM_PPM_h__

void ppmSetup(uint8_t pin, uint8_t channelCount);
void ppmDisable();
bool PPMEnabled();
void setPPMOutputChannelValue(uint8_t channel, int value);
void SUM_PPM_ISR();

#endif
