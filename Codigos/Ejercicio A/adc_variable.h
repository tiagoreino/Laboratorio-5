#ifndef ADC_VARIABLE_H_
#define ADC_VARIABLE_H_

#include <avr/io.h>

void ADC_init(uint8_t preescaler);

uint16_t ADC_leer(uint8_t canal);

#endif 