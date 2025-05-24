#include "main.h"
#include "delay.h"

// Delays for us microseconds
void delayMicroseconds(uint16_t us)
{

    TIM10->CNT = 0;
    uint16_t start = (uint16_t) TIM10->CNT;

    uint32_t duration = us * 60;	// Our MCU runs at 60 MHz, so each microsecond lasts 60 clock ticks
    while (TIM10->CNT - start < duration);

}
