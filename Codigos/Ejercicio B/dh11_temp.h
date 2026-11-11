#ifndef DH11_TEMP_H_
#define DH11_TEMP_H_

/* F_CPU va en las opciones del proyecto (Toolchain > Symbols: F_CPU=16000000UL),
   no acá: si otro .c lo define distinto, los _delay_ quedan mal. */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>

#define DHT_OK            0
#define DHT_ERR_TIMEOUT   1
#define DHT_ERR_CHECKSUM  2

void    DHT_start(void);
uint8_t DHT_response(void);
uint8_t DHT_leer(uint8_t *temperatura, uint8_t *humedad);
uint8_t obtener_temperatura_dht(void);   /* 99 si falla */

#endif