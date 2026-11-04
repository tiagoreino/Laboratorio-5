#ifndef DH11_TEMP_H_
#define DH11_TEMP_H_

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

void DHT_start();

uint8_t DHT_response();

uint8_t DHT_read(void);

uint8_t obtener_temperatura_dht(void);

#endif 