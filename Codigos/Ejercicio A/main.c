#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "lcd_con_i2c.h"
#include "adc_variable.h"
#include "dh11_temp.h" // DH11 = PD5

// ========== INDICE DE ESTADOS ==================
#define EST_ACTIVO      0
#define EST_STANDBY     1
#define EST_EMERGENCIA  3

#define Pot 0      // ADC0 = PC0
#define B1 PD2     // INT0 = PD2
#define B2 PD3     // INT1 = PD3
#define LED PD4
// DH11 = PD5
#define Bu PD6

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
int8_t valor_dh11 = 0;
uint8_t i = 0;
uint8_t estadoin = EST_ACTIVO;

int main(void){
	
	DDRD |= (1<<LED);
	
	ADC_init(128);
	lcd_init();
	configurar_interrupciones();
	
	estado = activo;
	
	while (1)
	{
		if (adc_pendiente)
		{
			adc_pendiente = 0;
			valor_adc = ADC_leer(Pot);
		}

		if (led_pendiente)
		{
			led_pendiente = 0;
			PORTD ^= (1 << LED);
		}

		if (temperatura_pendiente)
		{
			temperatura_pendiente = 0;
			valor_dh11 = obtener_temperatura_dht();
		}

		if (lcd_pendiente)
		{
			lcd_pendiente = 0;
			actualizar_lcd(estadoin);
		}
		
		if (evento_emergencia)
		{
			evento_emergencia = 0;
			estado = emergencia;
		}
		else if (evento_boton)
		{
			evento_boton = 0;
			estado = actividad;
		}

		estado();
	}
	
}

static void activo(void)
{
	PORTD = (0 << Bu);
	estadoin = EST_ACTIVO;
	
	if (contador_ms >= 10000)
	{
		estado = standby;
	}
	
}

static void standby(void)
{
	PORTD = (0 << Bu);
	estadoin = EST_STANDBY;	
	
	if (contador_ms >= 30000)
	{
		estado = apagado;
	}
	
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

void actualizar_lcd(uint8_t indice_estado)
{
	char linea1[17];
	char linea2[17];

	// Recuperar puntero almacenado en Flash
	const char *texto_flash = (const char *)pgm_read_ptr(&tabla_estados[indice_estado]);

	// Copiar titulo desde Flash hacia SRAM
	strcpy_P(linea1, texto_flash);
	
	if(indice_estado == 0){
		
		// Generar información dinámica
		snprintf_P(linea2, sizeof(linea2), PSTR("A:%u T:%dC"), (unsigned int)valor_adc, (int)valor_dh11);
		
	}else{
		
		const char *texto_flash = (const char *)pgm_read_ptr(&tabla_estados[indice_estado + 1]);
		
		// Copiar subtitulo desde Flash hacia SRAM
		strcpy_P(linea2, texto_flash);
		
	}
	lcd_titulo(linea1, linea2);
}

// ============ TIMER0 1ms ====================
volatile uint32_t contador_ms = 0;

ISR(TIMER0_COMPA_vect)
{
	contador_ms++;
}

// ============ TIMER1 100ms comprueba ADC y parpadea LED cada 200ms ====================
volatile uint8_t adc_pendiente = 0;
volatile uint8_t led_pendiente = 0;
volatile uint8_t lcd_pendiente = 0;
volatile uint8_t temperatura_pendiente = 0;

static uint8_t ticks_100ms = 0;

ISR(TIMER1_COMPA_vect)
{
	adc_pendiente = 1;

	ticks_100ms++;

	if ((ticks_100ms % 2) == 0)
	led_pendiente = 1;

	if ((ticks_100ms % 5) == 0)
	lcd_pendiente = 1;

	if (ticks_100ms >= 20)
	{
		ticks_100ms = 0;
		temperatura_pendiente = 1;
	}
}

//=================== VECTORES DE INTERRUPCIONES POR BOTONES ===================
volatile uint8_t evento_boton = 0;
volatile uint8_t evento_emergencia = 0;

ISR(INT0_vect)
{
	evento_boton = 1;
}

ISR(INT1_vect)
{
	evento_emergencia = 1;
}

const char texto_activo[] PROGMEM = "Activo";
const char texto_standby[] PROGMEM = "Standby";
const char texto_esperando[] PROGMEM = "Esperando...";
const char texto_emergencia[] PROGMEM = "EMERGENCIA";
const char texto_seguro[] PROGMEM = "ESTADO SEGURO";

const char* const tabla_estados[] PROGMEM = {
	texto_activo,     
	texto_standby,    
	texto_esperando,  
	texto_emergencia,
	texto_seguro
};