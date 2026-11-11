#ifndef ADC_VARIABLE_H_
#define ADC_VARIABLE_H_

#include <avr/io.h>

void ADC_init(uint8_t preescaler);

uint16_t ADC_leer(uint8_t canal);

/* Para PRR: el ADC se apaga en este orden (primero ADEN=0, después PRADC)
   y se prende al revés. Si se pone PRADC con ADEN=1 el ADC sigue consumiendo. */
void ADC_apagar(void);
void ADC_encender(void);

#endif