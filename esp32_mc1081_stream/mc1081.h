#ifndef MC1081_H
#define MC1081_H

#include <Arduino.h>
#include "mc1081_reg.h"

bool CAP_AFE_Transmit(uint8_t deviceAddr, uint8_t regAddr, uint8_t data);
bool CAP_AFE_Receive(uint8_t deviceAddr, uint8_t regAddr, uint8_t *pData, uint8_t size);
bool Registers_Init(MC1081_InitStructure *init);
bool MC1081_OSC2_Measure(CAP_AFE_DoubleEnded *cap, MC1081_InitStructure *init);

#endif
