#include <avr/io.h>
#include <avr/interrupt.h>

#include "lcd_con_i2c.h"
#include "adc_variable.h"

#define Pot 0      // ADC0 = PC0
#define B1 PD2     // INT0 = PD2
#define B2 PD3     // INT1 = PD3

// Puntero de estado actual
static void (*estado)(void);

// ========== Estados ==============
static void activo(void);
static void standby(void);
static void apagado(void);
static void actividad(void);
static void emergencia(void);

//=============== Configurar INT0 e INT1 ==================
void configurar_interrupciones(void);

int main(void){
	
	ADC_init(128);
	lcd_init();
	
	configurar_interrupciones();
	
	estado = activo;
	
	while(1){
		
		estado();
		
	}
	
}

static void activo(void)
{

	lcd_titulo("Activo", "A:"+ADC_leer(Pot)+"|T:"+"Lectura DHT"); //falta lectura dht
	
	
}

void configurar_interrupciones(void) {
	
	DDRD &= ~(1 << PD2) | ~(1 << PD3);  
	PORTD |= (1 << PD2) | (1 << PD3);   

	// Configurar el modo de disparo en EICRA
	// Flanco de bajada para ambos pines
	EICRA |= (1 << ISC01) | (1 << ISC11);
	EICRA &= ~(1 << ISC00) | ~(1 << ISC10);
	
	// Habilitar la interrupción externa en EIMSK
	EIMSK |= (1 << INT0) | (1 << INT1);

	// Habilitar las interrupciones globales
	sei();
}

//=================== VECTORES DE INTERRUPCIONES POR BOTONES ===================
ISR(INT0_vect) {
	actividad();
}

ISR(INT1_vect) {
	emergencia();
}