#include "dh11_temp.h"
#include <util/atomic.h>

#define DHT_PIN      PD5
#define DHT_LEER()   (PIND & (1 << DHT_PIN))
#define TIMEOUT_US   100   /* ningún pulso del DHT11 dura más de ~90 us */

/* Espera a que el pin llegue a 'nivel' (0 o distinto de 0).
   Devuelve 1 si llegó, 0 si se pasó el timeout -> no se cuelga nunca. */
static uint8_t esperar_nivel(uint8_t nivel)
{
	for (uint8_t i = 0; i < TIMEOUT_US; i++) {
		if ((DHT_LEER() != 0) == (nivel != 0)) return 1;
		_delay_us(1);
	}
	return 0;
}

void DHT_start(void)
{
	DDRD  |=  (1 << DHT_PIN);   /* salida */
	PORTD &= ~(1 << DHT_PIN);   /* bajo >= 18 ms */
	_delay_ms(20);
	PORTD |=  (1 << DHT_PIN);   /* soltar la línea */
	DDRD  &= ~(1 << DHT_PIN);   /* entrada con pull-up (PORTD sigue en 1) */
}

uint8_t DHT_response(void)
{
	/* el sensor tira la línea a bajo ~80 us, después alto ~80 us,
	   y después baja para empezar el primer bit */
	if (!esperar_nivel(0)) return 0;
	if (!esperar_nivel(1)) return 0;
	if (!esperar_nivel(0)) return 0;
	return 1;
}

/* Devuelve 0..255 en *byte; retorna 0 si hubo timeout */
static uint8_t DHT_read_byte(uint8_t *byte)
{
	uint8_t r = 0;
	for (uint8_t j = 0; j < 8; j++) {
		if (!esperar_nivel(1)) return 0;   /* fin del bajo de 50 us */
		_delay_us(40);                      /* '0' dura ~27 us, '1' ~70 us */
		r <<= 1;
		if (DHT_LEER()) {
			r |= 1;
			if (!esperar_nivel(0)) return 0;
		}
	}
	*byte = r;
	return 1;
}

uint8_t DHT_leer(uint8_t *temperatura, uint8_t *humedad)
{
	uint8_t d[5];
	uint8_t ok = 1;

	DHT_start();                 /* los 20 ms con interrupciones prendidas */

	/* los ~4 ms de bits son críticos en tiempo: sin interrupciones */
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if (!DHT_response()) ok = 0;
		for (uint8_t i = 0; ok && i < 5; i++) {
			if (!DHT_read_byte(&d[i])) ok = 0;
		}
	}
	if (!ok) return DHT_ERR_TIMEOUT;

	if ((uint8_t)(d[0] + d[1] + d[2] + d[3]) != d[4]) return DHT_ERR_CHECKSUM;

	*humedad     = d[0];
	*temperatura = d[2];
	return DHT_OK;
}

/* Compatibilidad con lo que ya usabas: 99 = error */
uint8_t obtener_temperatura_dht(void)
{
	uint8_t t, h;
	if (DHT_leer(&t, &h) != DHT_OK) return 99;
	return t;
}