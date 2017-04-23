#ifndef LED_H
#define LED_H

#include <p33FJ256GP710A.h>
#include "../Common.h"

#define LED_BLINK_LONG 1000
#define LED_BLINK_SHORT 250


void initLED(bool isAttMan);

void setLEDState(bool on);

void toggleLEDState();

void setLEDBrightness(uint8_t value);
#endif
