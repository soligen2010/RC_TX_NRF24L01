#include <arduino.h>

#ifndef __have__RC_RX_TX_UTIL_h__
#define __have__RC_RX_TX_UTIL_h__

void getChannelSequence (uint8_t outArray[], uint8_t numChannels, uint64_t permutation);
uint8_t getNextChannel (uint8_t seqArray[], uint8_t seqArraySize, uint8_t prevChannel);

#endif
