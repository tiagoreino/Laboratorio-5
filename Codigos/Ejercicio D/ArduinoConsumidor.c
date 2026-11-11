// Laboratorio 5 - Problema D. Consumidor ATmega328P, oscilador de 16 MHz.
// D8/PB0 -> ICP1 del medidor: pulso reservado al despertar por PD2.

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
/* Recompilar entre ensayos. F_CPU sigue siendo el oscilador de 16 MHz.
 * Para las esperas se usa la frecuencia efectiva, después de CLKPR. */
#ifndef DIVISOR_CPU
#define DIVISOR_CPU 1
#endif
#if F_CPU != 16000000UL
#error "F_CPU debe representar el oscilador de 16 MHz."
#endif
#if DIVISOR_CPU == 1
#define AJUSTE_CLKPR clock_div_1
#elif DIVISOR_CPU == 2
#define AJUSTE_CLKPR clock_div_2
#elif DIVISOR_CPU == 8
#define AJUSTE_CLKPR clock_div_8
#elif DIVISOR_CPU == 64
#define AJUSTE_CLKPR clock_div_64
#elif DIVISOR_CPU == 128
#define AJUSTE_CLKPR clock_div_128
#else
#error "DIVISOR_CPU debe ser 1, 2, 8, 64 o 128."
#endif
#define RELOJ_EFECTIVO (F_CPU / DIVISOR_CPU)
#define ESPERA_US(us) __builtin_avr_delay_cycles((RELOJ_EFECTIVO * (us##ULL) + 999999ULL) / 1000000ULL)

/* Ensayos puntuales por recompilación, sin alterar la secuencia de estados. */
#ifndef WDT_NOMINAL_MS
#define WDT_NOMINAL_MS 8000
#endif
#if WDT_NOMINAL_MS == 8000
#define PRESC_WDT (_BV(WDP3) | _BV(WDP0))
#elif WDT_NOMINAL_MS == 1000
#define PRESC_WDT (_BV(WDP2) | _BV(WDP1))
#elif WDT_NOMINAL_MS == 250
#define PRESC_WDT _BV(WDP2)
#elif WDT_NOMINAL_MS == 16
#define PRESC_WDT 0
#else
#error "WDT_NOMINAL_MS debe ser 16, 250, 1000 u 8000."
#endif

/* Configuración */
#define ANTIRREBOTE_MS 30
#define TAREA_MS 100
#define PRR_TODOS (_BV(PRADC) | _BV(PRUSART0) | _BV(PRSPI) | _BV(PRTIM0) \
                  | _BV(PRTIM1) | _BV(PRTIM2) | _BV(PRTWI))
/* Orden solicitado. Un bit en 1 bloquea el reloj del periférico. */
static const uint8_t bits_prr[] = {
    _BV(PRADC), _BV(PRUSART0), _BV(PRSPI), _BV(PRTIM0),
    _BV(PRTIM1), _BV(PRTIM2), _BV(PRTWI)
};

// Variables compartidas con las interrupciones 
static volatile uint8_t s1_pendiente, s2_pendiente, wdt_pendiente;

// Estado actual y acumulador de la tarea de CPU.
// Pasos pares Sleep e impares ACTIVO. Arranque en 0, IDLE.
static uint8_t paso, posicion_prr;
static volatile uint32_t trabajo;

// Funciones implementadas
static void configurar_wdt(uint8_t habilitado);
static void esperar_liberacion(uint8_t bit);
static uint8_t validar_pulsacion(uint8_t bit);
static void ejecutar_tarea(uint8_t duracion_ms);
static void preparar_estado(void);
static void cambiar_prr(void);
static void inicializar_perifericos(void);

// Programa principal
int main(void) {
    inicializar_perifericos();
    preparar_estado();
    EIFR = _BV(INTF0);
    EIMSK = _BV(INT0);
    sei();
    for (;;) {
        /* Prueba y sueño atómicos. AVR ejecuta SLEEP inmediatamente tras
         * SEI, incluso con IRQ pendiente. BOD se desactiva sólo al dormir.
         * Idle conserva el reloj principal; Power-Down lo detiene.
         * La primera acción observable después
         * de SLEEP e INT0 es subir D8, antes de PRR y del antirrebote.
         * D8 queda alto unos 64 us. ICP1 registra únicamente la subida. */
        if (!(paso & 1)) {
            cli();
            if (!s1_pendiente && !wdt_pendiente) {
                sleep_enable();
                sleep_bod_disable();
                sei();
                sleep_cpu();
                cli(); // Evita que WDT alargue el pulso de D8.
                // WDT sólo activa su propia bandera y no genera pulso.
                if (s1_pendiente) {
                    PORTB |= _BV(PB0);
                    ESPERA_US(64);
                    PORTB &= ~_BV(PB0);
                }
                sleep_disable();
                sei();
            } else sei();
        }
        if (s1_pendiente) {
            configurar_wdt(0);
            uint8_t pulsacion_valida = validar_pulsacion(PD2);
            esperar_liberacion(PD2);
            cli();
            s1_pendiente = 0; // Limpia la bandera de pulsación.
            if (pulsacion_valida) {
                // Al llegar al sexto paso, el siguiente vuelve a IDLE.
                paso = (paso + 1) % 6;
                preparar_estado();
            } else if (!(paso & 1)) configurar_wdt(1);
            EIFR = _BV(INTF0); // Limpia la bandera de INT0.
            EIMSK |= _BV(INT0); // Habilita nuevamente INT0.
            sei();
        } else if (paso & 1) {
            if (s2_pendiente) {
                uint8_t pulsacion_valida = validar_pulsacion(PD3);
                // Se retira la prueba anterior con el LED PB5 para que su
                // corriente no se superponga al cambio de PRR.
                if (pulsacion_valida && !s1_pendiente) cambiar_prr();
                esperar_liberacion(PD3);
                cli();
                s2_pendiente = 0; // Limpia la bandera de pulsación.
                EIFR = _BV(INTF1); // Limpia la bandera de INT1.
                EIMSK |= _BV(INT1); // Habilita nuevamente INT1.
                sei();
            } else ejecutar_tarea(TAREA_MS);
        } else if (wdt_pendiente) {
            cli();
            wdt_pendiente = 0;
            sei();
            /* Con 16 ms nominales se usa una ráfaga más corta. */
            ejecutar_tarea(WDT_NOMINAL_MS == 16 ? 4 : TAREA_MS);
        }
    }
}

// Configuración inicial: entradas S1/S2 con pull-up propio del DUT.
static void inicializar_perifericos(void) {
    cli();
    clock_prescale_set(AJUSTE_CLKPR);
	//Instrucción para dejar los pines que no usamos como salidas y en baja
    DDRB = 0xFF;
    PORTB = 0;
    DDRC = 0x3F;
    PORTC = 0;
	//Instrucción para utilizar los pines PD2 y PD3 como entradas y dejar PD4..PD7 como salidas en baja (PD0/PD1 sin uso)
    DDRD = 0b11110000;
    PORTD = 0b00001100;
	
    ADCSRA = 0;
    ACSR = _BV(ACD);
	//Siguiente instrucción -> Desactiva los relojes de los perifericos que no utilizamos,
	// directamente lo hacemos estableciendo 1 en cada bits correspondiente del registro PRR (Power Reduction Register)
    PRR = PRR_TODOS;
    EICRA = _BV(ISC11); // INT0 por nivel bajo, INT1 por flanco de bajada.
    EIFR = _BV(INTF0) | _BV(INTF1); // Limpia la bandera de INT0/INT1
    EIMSK = 0; // Se habilita INT0 después de preparar IDLE.
}

//Función principal para que el WDT no provoque un ciclo de reinicios cuando el microcontrolador carga el programa principal
// attribute -> extension del compilador GCC para asignar propiedades especiales a una función
// section(".init3") -> indica al compilador que coloquen esta funcion en usa seccion especial : init3 
//init3 -> seccion especial que forman parte de la inicializacion que se ejecuta antes del MAIN
//nake -> indica que esta esta función no se ejecute mediante una llamada normal , porque forma parte de la secuencia de arranque
void desactivar_wdt_arranque(void) __attribute__((naked, section(".init3")));
void desactivar_wdt_arranque(void) {
    MCUSR = 0; // limpia las banderas de MCU Status Register donde se encuentra el WDRF (esto es necesario para que el WDT no permanezca habilitado hasta la ejecución del main)
    wdt_disable(); // Macro de la librería del WDT, contiene la secuencia necesaria para deshabilitar el WDT
}

//Configuración del WDT dentro de una ventana de 4 ciclos del Reloj, manejo de interrupciones y proteccion de registros
static void configurar_wdt(uint8_t habilitado) {
    /* Calcular el valor antes de abrir la ventana de cuatro ciclos. */
    uint8_t valor = _BV(WDIF) | (habilitado ? _BV(WDIE) | PRESC_WDT : 0);
    uint8_t sreg_guardado = SREG; // Conserva el estado previo de la CPU e interrupciones.
    cli();
    wdt_reset();
    MCUSR &= ~_BV(WDRF);
    __asm__ __volatile__(
        "sts %[reg], %[abrir]\n\t"
        "sts %[reg], %[valor]"
        :
        : [reg] "n" (_SFR_MEM_ADDR(WDTCSR)),
          [abrir] "r" ((uint8_t)(_BV(WDCE) | _BV(WDE))), [valor] "r" (valor)
        : "memory");
    wdt_pendiente = 0;
    SREG = sreg_guardado; // Restaura el estado previo al cli().
}

//DEBOUNCE
static void esperar_liberacion(uint8_t bit)
{
	uint8_t n = 0;

	while (n < ANTIRREBOTE_MS)
	{
		if (bit == PD3 && s1_pendiente) // Instrucción necesaria para darle prioridad al cambio de estado si se encuentra en este Bucle
		{
			return;
		}

		ESPERA_US(1000);

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

//Delay _ validación del pulso
static uint8_t validar_pulsacion(uint8_t bit) {
    uint8_t n;
    for (n = 0; n < ANTIRREBOTE_MS; ++n) {
        ESPERA_US(1000);
        if ((bit == PD3 && s1_pendiente) || (PIND & _BV(bit))) {
			return 0;
		}
    }
    return 1;
}

//Tarea programada para la CPU
static void ejecutar_tarea(uint8_t duracion_ms) {
    uint8_t i;
    for (i = 0; i < duracion_ms; ++i) {
        trabajo = trabajo * 1664525UL + 1013904223UL; // Generador congruencial lineal (LCG), realiza operación matemática de 32bits con el microprocesador de 8 bits , por lo que representa una carga de trabajo considerable
        ESPERA_US(1000);
        if (s1_pendiente || s2_pendiente) { // S1 tiene prioridad sobre la tarea.
			break;
			}  
    }
}

//Condiciones necesarias dependiendo del estado en que se encuentre 
/* Entrar a un estado modifica sólo los despertadores necesarios.
 * Llamar con interrupciones deshabilitadas. El ADC apagado en ADC NR
 * evita conversiones automáticas y despertares ajenos al ensayo.
 * No hay timers periódicos habilitados ni siquiera en Idle. */
static void preparar_estado(void) {
    EIMSK &= ~_BV(INT1);
    s2_pendiente = 0;
    posicion_prr = 0;
    PRR = PRR_TODOS;
    configurar_wdt(!(paso & 1));
    EIFR = _BV(INTF1);
    if (paso & 1) EIMSK |= _BV(INT1);
    else if (paso == 0) set_sleep_mode(SLEEP_MODE_IDLE);
    else if (paso == 2) set_sleep_mode(SLEEP_MODE_ADC);
    else set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

/* Posiciones 1..7 individuales, 8..14 acumulativas. El cambio de PRR
 * sólo se observa en la corriente. D8 queda reservado al despertar. */
static void cambiar_prr(void) {
    posicion_prr = (posicion_prr + 1) % 15;
    uint8_t habilitados = 0;
    if (posicion_prr >= 8) {
        for (uint8_t i = 0; i < posicion_prr - 7; ++i) habilitados |= bits_prr[i];
    } else if (posicion_prr) habilitados = bits_prr[posicion_prr - 1];
    PRR = PRR_TODOS & ~habilitados;
}

// Rutinas de interrupción
// El nivel bajo despierta de Sleep; enmascarar para evitar ISR repetidas.
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

