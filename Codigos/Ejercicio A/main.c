#include <avr/io.h>
#include <avr/interrupt.h>

#include "lcd_con_i2c.h"
#include "adc_variable.h"

#define Pot 0      // ADC0 = PC0
#define B1 PD2     // INT0 = PD2
#define B2 PD3     // INT1 = PD3
#define LED PD4

// Puntero de estado actual
static void (*estado)(void);

// ========== Estados ==============
static void activo(void);
static void standby(void);
static void apagado(void);
static void actividad(void);
static void emergencia(void);

//=============== Configurar interrupciones ==================
void configurar_interrupciones(void);

uint16_t valor_adc = 0;
uint8_t i = 0;

int main(void){
	
	DDRD |= (1<<LED);
	
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

	lcd_titulo("Activo", ("A:%f | T:%iC", valor_adc, valor_temp)); //falta lectura dht

	
}

void configurar_interrupciones(void) {
	// Botones INT0 e INT1
	DDRD &= ~((1 << PD2) | (1 << PD3));
	PORTD |= (1 << PD2) | (1 << PD3);
	
	// Timer0 = Modo CTC 
	TCCR0A &= ~(1 << WGM00);
	TCCR0A |= (1 << WGM01);
	TCCR0B &= ~(1 << WGM02);
	
	// Se HABILITA la interrupción
	TIMSK0 |= (1 << OCIE0A);
	
	OCR0A = 249; // Para 16MHz con prescaler 64: 16M / (64 * 1000) = 250
	
	// Preescaler 64
	TCCR0B |= (1 << CS01) | (1 << CS00);
	TCCR0B &= ~(1 << CS02);
	
	// Timer1 = Modo CTC
	TCCR1A &= ~((1 << WGM11) | (1 << WGM10));
	TCCR1B &= ~(1 << WGM13);
	TCCR1B |= (1 << WGM12);
	
	// Se HABILITA la interrupción
	TIMSK1 |= (1 << OCIE1A);
	
	OCR1A = 24999; // Para 16MHz con prescaler 64: 16M / (64 * 10) = 25000
	
	// Preescaler 64
	TCCR1B |= (1 << CS11) | (1 << CS10);
	TCCR1B &= ~(1 << CS12);

	// Configurar interrupciones externas (Flanco de bajada)
	EICRA |= (1 << ISC01) | (1 << ISC11);
	EICRA &= ~((1 << ISC00) | (1 << ISC10));
	
	EIMSK |= (1 << INT0) | (1 << INT1);

	sei(); // Habilitar globales
}

// ============ TIMER0 1ms ====================
volatile uint32_t contador_ms = 0;

ISR(TIMER0_COMPA_vect)
{
	contador_ms++;
}

// ============ TIMER1 100ms comprueba ADC y parpadea LED cada 200ms ====================
ISR(TIMER1_COMPA_vect) {
	
	valor_adc = ADC_leer(Pot);
	i++;

	if (i >= 2)
	{
		PORTD ^= (1 << LED);
		i = 0;
	}
}

//=================== VECTORES DE INTERRUPCIONES POR BOTONES ===================
ISR(INT0_vect) {
	estado = actividad;
}

ISR(INT1_vect) {
	estado = emergencia;
}