#ifndef DH11_TEMP_H_
#define DH11_TEMP_H_

#include <avr/io.h>

void DHT_start();

uint8_t DHT_response();

uint8_t obtener_temperatura_dht(void);

#endif 