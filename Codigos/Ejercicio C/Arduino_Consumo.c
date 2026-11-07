// Laboratorio 5 - Problema C

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

/* Configuración */
#define ANTIRREBOTE_MS 30
#define TAREA_MS 100
enum { A_ACTIVO, B_IDLE, C_WDT, D_PROFUNDO };

// Variables compartidas con las interrupciones 
static volatile uint8_t s1_pendiente, s2_pendiente, wdt_pendiente;

// Estado actual y acumulador de la tarea de CPU.
static uint8_t estado = A_ACTIVO;
static volatile uint32_t trabajo;

// Funciones implementadas
static void configurar_wdt(uint8_t habilitado);
static void esperar_liberacion(uint8_t bit);
static uint8_t validar_pulsacion(uint8_t bit);
static void ejecutar_tarea(void);
static void preparar_estado(void);
static void dormir_hasta_evento(void);
static void inicializar_perifericos(void);

// Programa principal
int main(void) {
    inicializar_perifericos();
    preparar_estado();
    while(1) {
        if (s1_pendiente) {
            configurar_wdt(0);
            if (validar_pulsacion(PD2)) {
				estado = (estado + 1) & 3; //Implementación para lograr que al llegar al cuarto (D_PROFUNDO) estado el siguiente estado sea el 0 (A_ACTIVO)
			}
            esperar_liberacion(PD2);
			cli();                 
			s1_pendiente = 0;       // Limpia la bandera de pulsación
			EIFR = _BV(INTF0);      // Limpia la bandera de INT0
			EIMSK |= _BV(INT0);     // Habilita nuevamente INT0
			sei();                 
            preparar_estado();
        } 
		else if (estado == A_ACTIVO) {
			ejecutar_tarea();
		}
        else if (estado == B_IDLE && s2_pendiente ) {
            uint8_t pulsacion_valida = validar_pulsacion(PD3);
            esperar_liberacion(PD3);
            cli();
            s2_pendiente = 0;	// Limpia la bandera de pulsación
            EIFR = _BV(INTF1);	// Limpia la bandera de INT1
            EIMSK |= _BV(INT1);	// Habilita nuevamente INT1
            sei();
            if (pulsacion_valida) {
				ejecutar_tarea();
				// Prueba: encender LED PB5 durante 500 ms si no está accionado PD2, para observar cambios en la corriente
				if (!s1_pendiente) {
					PORTB |= _BV(PB5);
					_delay_ms(500);
					PORTB &= ~_BV(PB5);
				}
			}
        } 
		else if (estado == C_WDT && wdt_pendiente) {
            cli();
            wdt_pendiente = 0;
            sei();
            ejecutar_tarea();
        } 
		else {
			dormir_hasta_evento();
		}
    }
}