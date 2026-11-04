#define F_CPU 16000000UL

// Librerias del sistema
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <util/atomic.h>
#include <stdio.h>

// Librerias propias
#include "lcd_con_i2c.h"
#include "adc_variable.h"
#include "dh11_temp.h" // DH11 = PD5

// =================== LUT ==================
const char texto_activo[] PROGMEM = "Estado: Activo";
const char texto_standby[] PROGMEM = "Estado: Standby";
const char texto_esperando[] PROGMEM = "Esperando...";
const char texto_emergencia[] PROGMEM = "EMERGENCIA";
const char texto_seguro[] PROGMEM = "Estado: Seguro";

const char* const tabla_estados[] PROGMEM = {
	texto_activo,
	texto_standby,
	texto_esperando,
	texto_emergencia,
	texto_seguro
};

// ========== INDICE DE ESTADOS ==================
#define EST_ACTIVO      0
#define EST_STANDBY     1
#define EST_APAGADO     2
#define EST_EMERGENCIA  3

#define Pot 0      // ADC0 = PC0
#define B1 PD2     // INT0 = PD2
#define B2 PD3     // INT1 = PD3
#define LED PD4
// DH11 = PD5
#define Bu PD6

// Puntero de estado actual
static void (*estado)(void);

static void cambiar_estado(void (*nuevo)(void));

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
static uint8_t entrando = 1;

uint32_t obtener_tiempo(void);
void actualizar_lcd(uint8_t indice_estado);
void configurar_interrupciones(void);

// ===== VARIABLES GLOBALES =====

volatile uint8_t estadoin = EST_ACTIVO;

volatile uint32_t contador_ms = 0;

volatile uint8_t adc_pendiente = 0;
volatile uint8_t led_pendiente = 0;
volatile uint8_t lcd_pendiente = 0;
volatile uint8_t temperatura_pendiente = 0;

volatile uint8_t evento_boton = 0;
volatile uint8_t evento_emergencia = 0;

static uint8_t ticks_100ms = 0;

int main(void){
	
	DDRD |= (1 << LED) | (1 << Bu);
	
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
			cambiar_estado(emergencia);
		}
		else if (evento_boton)
		{
			evento_boton = 0;
			if (estado == apagado)
			{
				lcd_power_on();      // solo si venía apagada
			}
			cambiar_estado(actividad);
		}

		estado();
	}
	
}

static void activo(void)
{
	if (entrando)
	{
		entrando = 0;
		estadoin = EST_ACTIVO;
		PORTD &= ~(1 << Bu);
		actualizar_lcd(EST_ACTIVO);       // muestra "Activo" al instante
	}

	if (obtener_tiempo() >= 10000)
	{
		cambiar_estado(standby);
	}
}

static void standby(void)
{
	if (entrando)
	{
		entrando = 0;
		estadoin = EST_STANDBY;
		lcd_pendiente = 0;                // descarta un refresco "Activo" pendiente
		PORTD &= ~((1 << Bu) | (1 << LED));
		actualizar_lcd(EST_STANDBY);      // "Estado: Standby" + "Esperando..." UNA vez
	}

	// Faltaria Dormir --> Parte B

	if (obtener_tiempo() >= 30000)
	{
		cambiar_estado(apagado);
	}
}

static void apagado(void)
{
	if (entrando)
	{
		entrando = 0;
		estadoin = EST_APAGADO;
		lcd_pendiente = 0;
		PORTD &= ~((1 << Bu) | (1 << LED));
		lcd_power_off();                  // UNA vez, no en cada vuelta
	}

	// Faltaria Dormir --> Parte B
}

static void actividad(void)
{
	if (entrando)
	{
		entrando = 0;
		estadoin = EST_ACTIVO;
	}

	if (!(PIND & (1 << B1)))
	{
		PORTD |= (1 << Bu);               // Buzzer encendido
	}
	else
	{
		PORTD &= ~(1 << Bu);              // Buzzer apagado
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			contador_ms = 0;
		}
		cambiar_estado(activo);
	}
}

static void emergencia(void)
{
	cli();
	lcd_power_on();                       // por si venía de APAGADO con el display off
	actualizar_lcd(EST_EMERGENCIA);
	PORTD &= ~((1 << Bu) | (1 << LED));
	estadoin = EST_EMERGENCIA;

	while (1)
	{
		// Permanece bloqueado hasta reiniciar el microcontrolador.
	}
}

static void cambiar_estado(void (*nuevo)(void))
{
	estado = nuevo;
	entrando = 1;
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
		snprintf_P(linea2, sizeof(linea2), PSTR("ADC:%u Tem:%dC"), (unsigned int)valor_adc, (int)valor_dh11);
		
	}else{
		
		const char *texto_flash = (const char *)pgm_read_ptr(&tabla_estados[indice_estado + 1]);
		
		// Copiar subtitulo desde Flash hacia SRAM
		strcpy_P(linea2, texto_flash);
		
	}
	lcd_titulo(linea1, linea2);
}

// ============ TIMER0 1ms ====================


ISR(TIMER0_COMPA_vect)
{
	contador_ms++;
}

uint32_t obtener_tiempo(void)
{
	uint32_t tiempo;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		tiempo = contador_ms;
	}

	return tiempo;
}

// ============ TIMER1 100ms comprueba acciones mediante flags ====================

ISR(TIMER1_COMPA_vect)
{
	if (estadoin == EST_ACTIVO)
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
}
//=================== VECTORES DE INTERRUPCIONES POR BOTONES ===================

ISR(INT0_vect)
{
	evento_boton = 1;
}

ISR(INT1_vect)
{
	evento_emergencia = 1;
}

