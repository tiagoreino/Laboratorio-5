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

// Configuraci�n inicial: entradas S1/S2 con pull-up propio del DUT.
static void inicializar_perifericos(void) {
    cli(); 
	//Instrucci�n para dejar los pines que no usamos como salidas y en baja
    DDRB = 0xFF;
    PORTB = 0;
    DDRC = 0x3F;
    PORTC = 0;
	//Instrucci�n para utilizar los pines PD2 y PD3 como entradas y dejar los dem�s pines PD como salidas en baja
    DDRD = 0b11110000;
    PORTD = 0b00001100;
	
    ADCSRA = 0;
    ACSR = _BV(ACD);
	//Siguiente instrucci�n -> Desactiva los relojes de los perifericos que no utilizamos,
	// directamente lo hacemos estableciendo 1 en cada bits correspondiente del registro PRR (Power Reduction Register)
    PRR = _BV(PRADC) | _BV(PRUSART0) | _BV(PRSPI) | _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRTIM2); 
    EICRA = 0; // INT0/INT1 por nivel bajo. 
    EIFR = _BV(INTF0) | _BV(INTF1); // Limpia la bandera de INT0/INT1
    EIMSK = _BV(INT0);
    sei();
}

//Funci�n principal para que el WDT no provoque un ciclo de reinicios cuando el microcontrolador carga el programa principal
// attribute -> extension del compilador GCC para asignar propiedades especiales a una funci�n
// section(".init3") -> indica al compilador que coloquen esta funcion en usa seccion especial : init3 
//init3 -> seccion especial que forman parte de la inicializacion que se ejecuta antes del MAIN
//nake -> indica que esta esta funci�n no se ejecute mediante una llamada normal , porque forma parte de la secuencia de arranque
void desactivar_wdt_arranque(void) __attribute__((naked, section(".init3")));
void desactivar_wdt_arranque(void) {
    MCUSR = 0; // limpia las banderas de MCU Status Register donde se encuentra el WDRF (esto es necesario para que el WDT no permanezca habilitado hasta la ejecuci�n del main)
    wdt_disable(); // Macro de la librer�a del WDT, contiene la secuencia necesaria para deshabilitar el WDT
}

//Configuraci�n del WDT dentro de una ventana de 4 ciclos del Reloj, manejo de interrupciones y proteccion de registros
static void configurar_wdt(uint8_t habilitado) {
    uint8_t sreg_guardado = SREG; //SREG -> registro de estado de la CPU, guarda resultados de operaciones de la CPU y estado de las Interrupciones
    cli();
    wdt_reset();
    MCUSR &= ~_BV(WDRF);
	if(habilitado) 
	{
		WDTCSR = _BV(WDCE) | _BV(WDE);// Habilita la cofiguracion del WDT
		WDTCSR = (_BV(WDIE) | _BV(WDP3) | _BV(WDP0)); //Configuracion del WDT (interrupcion_WDT habilitada con WDIE | reset_WDT deshabilitada usando WDE | tiempo_WDT 8 segundos con WDP3 y WDP0)
	}
	else{
		wdt_disable();
	}
    wdt_pendiente = 0;
    SREG = sreg_guardado; //Reestablecemos el valor del registro SREG para la restauracion del estado previo a ejecutar el cli()
}

//DEBOUNCE
static void esperar_liberacion(uint8_t bit)
{
	uint8_t n = 0;

	while (n < ANTIRREBOTE_MS)
	{
		if (bit == PD3 && s1_pendiente) // Instruccion necesaria para darle prioridad al cambio de estado si se encuentra en este Bucle
		{
			return;
		}

		_delay_ms(1);

		if (PIND & _BV(bit))
		{
			++n;
		}
		else
		{
			n = 0;
		}
	}
}

//Delay _ validaci�n del pulso
static uint8_t validar_pulsacion(uint8_t bit) {
    uint8_t n;
    for (n = 0; n < ANTIRREBOTE_MS; ++n) {
        _delay_ms(1);
        if (PIND & _BV(bit)) {
			return 0;
		}
    }
    return 1;
}

//Tarea programada para la CPU
static void ejecutar_tarea(void) {
    uint8_t i;
    for (i = 0; i < TAREA_MS; ++i) {
        trabajo = trabajo * 1664525UL + 1013904223UL; // Generador congruencial lineal (LCG), realiza operaci�n matem�tica de 32bits con el microprocesador de 8 bits , por lo que representa una carga de trabajo considerable
        _delay_ms(1);
        if (s1_pendiente) { // S1 tiene prioridad sobre la tarea.
			break;
			}  
    }
}

//Condiciones necesarias dependiendo del estado en que se encuentre 
static void preparar_estado(void) {
    cli();
    EIMSK &= ~_BV(INT1);
    s2_pendiente = 0;
    configurar_wdt(estado == C_WDT);
    EIFR = _BV(INTF1);
    if (estado == B_IDLE) {
		EIMSK |= _BV(INT1);
	}
    sei();
}

//Aplicando MODO_SLEEP
static void dormir_hasta_evento(void) {
    if (estado == B_IDLE)
    {
	    set_sleep_mode(SLEEP_MODE_IDLE);
    }
    else
    {
	    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    }
    cli();
    if (!s1_pendiente && !s2_pendiente && !wdt_pendiente) {
        sleep_enable();
        sleep_bod_disable();
        sei();
        sleep_cpu();
        sleep_disable();
    } else sei();
}

// Rutinas de interrupcion
ISR(INT0_vect) {
    EIMSK &= ~_BV(INT0);
    s1_pendiente = 1;
}

ISR(INT1_vect) {
    EIMSK &= ~_BV(INT1);
    s2_pendiente = 1;
}

ISR(WDT_vect) {
    wdt_pendiente = 1;
}

