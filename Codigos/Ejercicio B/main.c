#define F_CPU 16000000UL

// Librerias del sistema
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <string.h>
#include <util/atomic.h>
#include <stdio.h>

// Librerias propias
#include "lcd_con_i2c.h"
#include "adc_variable.h"
#include "dh11_temp.h" // DH11 = PD5
#include "bluetooth.h"

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

// DIP: SW1..SW6 = PB0..PB5, SW7 = PD7, SW8 = PC1 (ON = cerrado a GND)
// SW1 ADC, SW2 USART, SW3 SPI, SW4 TIM1, SW5 TIM0, SW6 TIM2, SW7 TWI, SW8 modo WDT

// PRR que se puede aplicar en cada estado (indice = estadoin)
const uint8_t prr_estado[4] = {
	(1<<PRSPI) | (1<<PRTIM2),                                                    // Activo / Actividad
	(1<<PRSPI) | (1<<PRTIM1) | (1<<PRTIM2) | (1<<PRADC),                         // Standby
	(1<<PRADC) | (1<<PRUSART0) | (1<<PRTWI) | (1<<PRTIM0) | (1<<PRSPI) | (1<<PRTIM1) | (1<<PRTIM2), // Apagado
	(1<<PRADC) | (1<<PRTIM0) | (1<<PRSPI) | (1<<PRTIM1) | (1<<PRTIM2)            // Seguro
};

// Puntero de estado actual
static void (*estado)(void);

static void cambiar_estado(void (*nuevo)(void));

// ========== Estados ==============
static void activo(void);
static void standby(void);
static void apagado(void);
static void actividad(void);
static void emergencia(void);

// Estados que se pueden pedir por BT (numeracion del diagrama)
static void (* const tabla_bt[5])(void) = { activo, standby, apagado, actividad, emergencia };

//=============== Configurar interrupciones ==================
void configurar_interrupciones(void);

uint16_t valor_adc = 0;
int8_t valor_dh11 = 0;
uint8_t i = 0;
static uint8_t entrando = 1;

uint32_t obtener_tiempo(void);
void actualizar_lcd(uint8_t indice_estado);
void configurar_interrupciones(void);
void entrar(uint8_t est);
void ejecutar(uint8_t reg_cmd);

// ===== VARIABLES GLOBALES =====

volatile uint8_t estadoin = EST_ACTIVO;

volatile uint32_t contador_ms = 0;
volatile uint8_t paso_ms = 1;

volatile uint8_t adc_pendiente = 0;
volatile uint8_t led_pendiente = 0;
volatile uint8_t lcd_pendiente = 0;
volatile uint8_t temperatura_pendiente = 0;

volatile uint8_t evento_boton = 0;
volatile uint8_t evento_emergencia = 0;
volatile uint8_t evento_wdt = 0;
volatile uint8_t por_nivel = 0;

static uint8_t ticks_100ms = 0;

#define T_BUZZER_BT 1000          // ms que suena el buzzer cuando la actividad viene del celular
static uint8_t actividad_bt = 0;

// Banderas que maneja la app
uint8_t led_on = 1, adc_on = 1, dht_on = 1, lcd_on = 1, eventos_on = 1;

// Registro de alarmas
uint8_t reg_tipo[8], reg_dato[8], reg_pos = 0, reg_cant = 0;

// Energia y WDT
uint8_t prr_permitido = 0;
uint8_t modo_reset = 0;        // SW8: 0 = prueba A (despertador), 1 = prueba B (reset)
uint8_t reset_wdt = 0;
uint8_t despertares_wdt = 0;

// Causa del reset: se lee antes del main para que el WDT no vuelva a resetear
uint8_t mcusr_copia __attribute__((section(".noinit")));
uint16_t marca_bloqueo __attribute__((section(".noinit")));
void leer_mcusr(void) __attribute__((naked, used, section(".init3")));
void leer_mcusr(void)
{
	uint8_t r2_boot;
	__asm__ __volatile__ ("mov %0, r2" : "=r" (r2_boot));   // Optiboot deja aca el MCUSR original
	mcusr_copia = MCUSR ? MCUSR : r2_boot;                  // sin bootloader MCUSR viene con datos
	MCUSR = 0;
	wdt_disable();
}
static void marcar_actividad(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		contador_ms = 0;
	}
}

static void notificar(uint8_t ev, uint8_t dato)
{
	if (eventos_on) bt_enviar(RSP_EVENTO | ev, dato);
}

static void registrar(uint8_t ev, uint8_t dato)
{
	reg_tipo[reg_pos] = ev;
	reg_dato[reg_pos] = dato;
	reg_pos = (reg_pos + 1) & 7;
	if (reg_cant < 8) reg_cant++;
	notificar(ev, dato);
}

static uint8_t leer_dip(void)
{
	DDRB &= ~0x3F;        PORTB |= 0x3F;
	DDRD &= ~(1 << PD7);  PORTD |= (1 << PD7);
	DDRC &= ~(1 << PC1);  PORTC |= (1 << PC1);
	_delay_us(20);

	uint8_t sw = ~PINB & 0x3F;
	if (!(PIND & (1 << PD7))) sw |= (1 << 6);
	if (!(PINC & (1 << PC1))) sw |= (1 << 7);

	// Salida en 0: no consume con el switch abierto ni cerrado
	PORTB &= ~0x3F;       DDRB |= 0x3F;
	PORTD &= ~(1 << PD7); DDRD |= (1 << PD7);
	PORTC &= ~(1 << PC1); DDRC |= (1 << PC1);
	return sw;
}

// SW1..SW4 -> bits 0..3, SW5..SW7 -> bits 5..7 (bit 4 reservado)
static uint8_t dip_a_prr(uint8_t sw)
{
	return (sw & 0x0F) | ((sw & 0x70) << 1);
}

static void aplicar_prr(uint8_t est)
{
	prr_permitido = dip_a_prr(leer_dip());
	uint8_t m = prr_estado[est] & prr_permitido;

	if (m & (1 << PRADC)) ADC_apagar();      else ADC_encender();
	if (m & (1 << PRUSART0)) bt_apagar();    else bt_prender();

	if (m & (1 << PRTWI)) {
		TWCR = 0;
		PRR |= (1 << PRTWI);
		} else if (PRR & (1 << PRTWI)) {
		PRR &= ~(1 << PRTWI);
		twi_init();
	}

	uint8_t timers = (1 << PRTIM0) | (1 << PRTIM1) | (1 << PRTIM2) | (1 << PRSPI);
	PRR = (PRR & ~timers) | (m & timers);
}

static void timer0_1ms(void)
{
	TCCR0B = 0;
	OCR0A = 249;                    // 16M / (64 * 250) = 1 ms
	paso_ms = 1;
	TCCR0B = (1 << CS01) | (1 << CS00);
}

static void timer0_10ms(void)
{
	TCCR0B = 0;
	OCR0A = 155;                    // 16M / (1024 * 156) = 9,98 ms
	paso_ms = 10;
	TCCR0B = (1 << CS02) | (1 << CS00);
}

static uint16_t leer_pot(void)
{
	uint8_t apagado_prr = PRR & (1 << PRADC);
	if (apagado_prr) ADC_encender();
	uint16_t v = ADC_leer(Pot);
	if (apagado_prr) ADC_apagar();
	return v;
}

static uint8_t hay_trabajo(void)
{
	return adc_pendiente || led_pendiente || lcd_pendiente || temperatura_pendiente
	|| evento_boton || evento_emergencia || evento_wdt || uart_hay();
}

static void botones_flanco(void)
{
	EIMSK = 0;
	EICRA = (1 << ISC01) | (1 << ISC11);
	EIFR = (1 << INTF0) | (1 << INTF1);
	por_nivel = 0;
	EIMSK = (1 << INT0) | (1 << INT1);
}

static void botones_nivel(void)      // en Power-Down INT0/INT1 solo despiertan por nivel
{
	EIMSK = 0;
	EICRA = 0;
	EIFR = (1 << INTF0) | (1 << INTF1);
	por_nivel = 1;
	EIMSK = (1 << INT0) | (1 << INT1);
}

static void dormir(uint8_t modo)
{
	set_sleep_mode(modo);
	cli();
	if (!hay_trabajo())
	{
		sleep_enable();
		sei();
		sleep_cpu();
		sleep_disable();
	}
	sei();
}

static void wdt_modo_b(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		wdt_reset();
		WDTCSR = (1 << WDCE) | (1 << WDE);
		WDTCSR = (1 << WDIE) | (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);   // 2 s: primero interrupcion, despues reset
	}
}

static void simular_bloqueo(void)
{
	lcd_titulo("SIMULANDO", "BLOQUEO...");
	while (1)
	{
		// No se llama a ereset() -> el WDT resetea a los 2 s
	}
}

int main(void){
	
	DDRD |= (1 << LED) | (1 << Bu);
	DIDR0 = (1 << ADC0D);

	modo_reset = leer_dip() >> 7;
	
	ADC_init(128);
	lcd_init();
	uart_init();

	configurar_interrupciones();

	if (modo_reset) wdt_modo_b();

	reset_wdt = (marca_bloqueo == 0xB10C);
	marca_bloqueo = 0;
	registrar(EV_RESET, mcusr_copia);

	if (reset_wdt || !(PIND & (1 << B2)))
	{
		estado = emergencia;                 // arranca directo en estado seguro
		}else{
		
		estado = activo;
		
	}

	while (1)
	{
		if (modo_reset) wdt_reset();

		uint8_t cmd;
		while (parser_tarea(&cmd, (uint16_t)obtener_tiempo()))
		{
			ejecutar(cmd);
		}

		if (adc_pendiente)
		{
			adc_pendiente = 0;
			if (adc_on) valor_adc = ADC_leer(Pot);
		}

		if (led_pendiente)
		{
			led_pendiente = 0;
			if (led_on) PORTD ^= (1 << LED);
			else PORTD &= ~(1 << LED);
		}

		if (temperatura_pendiente)
		{
			temperatura_pendiente = 0;
			if (dht_on)
			{
				valor_dh11 = obtener_temperatura_dht();
				if (valor_dh11 == 99) notificar(EV_ERROR_DHT, 0);
			}
		}

		if (lcd_pendiente)
		{
			lcd_pendiente = 0;
			if (lcd_on) actualizar_lcd(estadoin);
		}
		
		if (evento_emergencia)
		{
			evento_emergencia = 0;
			if (!(PIND & (1 << B2)) && estado != emergencia)          // solo si B2 sigue apretado
			{
				registrar(EV_EMERGENCIA, 0);
				cambiar_estado(emergencia);
			}
		}
		else if (evento_boton)
		{
			evento_boton = 0;
			if (estado != emergencia)
			{
				notificar(EV_BOTON, 0);
				actividad_bt = 0;     // si se aprieta B1 manda el boton
				cambiar_estado(actividad);
			}
		}

		estado();
	}
	
}

// Acciones comunes al entrar a un estado
void entrar(uint8_t est)
{
	entrando = 0;
	estadoin = est;
	aplicar_prr(est);
	if (est == EST_STANDBY) timer0_10ms();
	else timer0_1ms();
	if (est != EST_APAGADO) notificar(EV_CAMBIO_ESTADO, est);
}

static void activo(void)
{
	if (entrando)
	{
		entrar(EST_ACTIVO);
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
		entrar(EST_STANDBY);
		lcd_pendiente = 0;                // descarta un refresco "Activo" pendiente
		PORTD &= ~((1 << Bu) | (1 << LED));
		actualizar_lcd(EST_STANDBY);      // "Estado: Standby" + "Esperando..." UNA vez
	}

	if (obtener_tiempo() >= 30000)
	{
		cambiar_estado(apagado);
		return;
	}

	dormir(SLEEP_MODE_IDLE);              // despierta con Timer0, USART, INT0 o INT1
}

static void apagado(void)
{
	if (entrando)
	{
		notificar(EV_CAMBIO_ESTADO, EST_APAGADO);   // antes de cortar el BT
		lcd_power_off();                  // UNA vez, no en cada vuelta
		entrar(EST_APAGADO);
		lcd_pendiente = 0;
		PORTD &= ~((1 << Bu) | (1 << LED));
		despertares_wdt = 0;
		if (!modo_reset)                  // Prueba A: interrupcion cada 8 s
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				wdt_reset();
				WDTCSR = (1 << WDCE) | (1 << WDE);
				WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
			}
		}
	}

	if (evento_wdt)
	{
		evento_wdt = 0;
		despertares_wdt++;
		PORTD |= (1 << LED);              // pulso para medir el periodo
		valor_adc = leer_pot();
		_delay_ms(25);
		PORTD &= ~(1 << LED);
	}

	botones_nivel();
	if (modo_reset) wdt_disable();        // si no, el WDT resetea durante el sueño
	dormir(SLEEP_MODE_PWR_DOWN);
	if (modo_reset) wdt_modo_b();
	botones_flanco();
}

static void actividad(void)
{
	static uint32_t t_presionado = 0;

	if (entrando)
	{
		entrar(EST_ACTIVO);
		t_presionado = obtener_tiempo();
	}
	
	if (actividad_bt)
	{
		PORTD |= (1 << Bu);
		if (obtener_tiempo() - t_presionado >= T_BUZZER_BT)
		{
			actividad_bt = 0;
			PORTD &= ~(1 << Bu);
			marcar_actividad();
			cambiar_estado(activo);
		}
		return;
	}

	if (!(PIND & (1 << B1)))
	{
		PORTD |= (1 << Bu);               // Buzzer encendido
		if (modo_reset && obtener_tiempo() - t_presionado >= 5000)
		{
			simular_bloqueo();            // Prueba B: B1 mantenido 5 s
		}
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
	if (entrando)
	{
		entrar(EST_EMERGENCIA);
		PORTD &= ~((1 << Bu) | (1 << LED));
		if (!lcd_on) { lcd_on = 1; lcd_power_on(); }
		actualizar_lcd(EST_EMERGENCIA);
		if (reset_wdt)
		{
			lcd_set_cursor(0, 1);
			lcd_print("Reset por WDT  ");
		}
	}

	// Recuperacion solo local: B2 suelto y B1 apretado
	if ((PIND & (1 << B2)) && !(PIND & (1 << B1)))
	{
		reset_wdt = 0;
		registrar(EV_FIN_EMERG, 0);
		evento_boton = 0;
		marcar_actividad();
		cambiar_estado(activo);
	}
}

static void cambiar_estado(void (*nuevo)(void))
{
	if (estado == apagado && nuevo != apagado)
	{
		// Salir de APAGADO: devolver TWI y USART antes de usarlos
		if (!modo_reset) wdt_disable();
		evento_wdt = 0;
		PRR &= ~((1 << PRTWI) | (1 << PRTIM0) | (1 << PRTIM1));
		twi_init();
		bt_prender();
		if (lcd_on) lcd_power_on();
		notificar(EV_WDT, despertares_wdt);
	}
	estado = nuevo;
	entrando = 1;
}

void ejecutar(uint8_t reg_cmd){
	uint8_t cmd_alta = reg_cmd >> 4;
	uint8_t cmd_baja = reg_cmd & 0x0F;
	uint8_t seguro = (estado == emergencia);

	if (!(cmd_alta == 4 && cmd_baja == 0)) marcar_actividad();   // el ping no cuenta como actividad

	switch (cmd_alta){

		case (0):   // parpadeo LED
		case (1):   // ADC
		case (2):   // DHT
		case (3):   // LCD
		case (6):   // eventos
		if (cmd_baja > 1) { bt_enviar(RSP_NACK | ERR_DATO, reg_cmd); return; }
		if (seguro && (cmd_alta == 0 || cmd_alta == 3)) { bt_enviar(RSP_NACK | ERR_RECHAZADO, reg_cmd); return; }
		if (cmd_alta == 0) led_on = cmd_baja;
		if (cmd_alta == 1) adc_on = cmd_baja;
		if (cmd_alta == 2) dht_on = cmd_baja;
		if (cmd_alta == 3)
		{
			lcd_on = cmd_baja;
			if (lcd_on) { lcd_power_on(); actualizar_lcd(estadoin); }
			else lcd_power_off();
		}
		if (cmd_alta == 6) eventos_on = cmd_baja;
		bt_enviar(RSP_ACK | cmd_alta, estadoin);
		if (cmd_alta == 6 && cmd_baja == 1)
		{
			for (uint8_t k = 0; k < reg_cant; k++)
			{
				uint8_t j = (reg_pos + 7 - k) & 7;
				bt_enviar(RSP_REGISTRO | reg_tipo[j], reg_dato[j]);
			}
		}
		break;

		case (4):   // 1 = pedir datos, 0 = ping
		if (cmd_baja > 1) { bt_enviar(RSP_NACK | ERR_DATO, reg_cmd); return; }
		bt_enviar(RSP_DATO | DATO_ESTADO, estadoin);
		if (cmd_baja == 1)
		{
			bt_enviar(RSP_DATO | DATO_ADC, leer_pot() >> 2);
			bt_enviar(RSP_DATO | DATO_TEMP, valor_dh11);
			bt_enviar(RSP_DATO | DATO_PRR, PRR);
			bt_enviar(RSP_DATO | DATO_WDT, despertares_wdt);
		}
		break;

		case (5):   // cambiar de estado (0..4 como en el diagrama)
		if (cmd_baja > 4) { bt_enviar(RSP_NACK | ERR_DATO, reg_cmd); return; }
		if (seguro) { bt_enviar(RSP_NACK | ERR_RECHAZADO, reg_cmd); return; }
		bt_enviar(RSP_ACK | cmd_alta, cmd_baja);
		if (cmd_baja == 4) registrar(EV_EMERGENCIA, 1);
		if (cmd_baja == 3) actividad_bt = 1;
		cambiar_estado(tabla_bt[cmd_baja]);
		break;

		case (7):   // emergencia
		if (cmd_baja == 1)
		{
			bt_enviar(RSP_ACK | cmd_alta, estadoin);
			if (!seguro) { registrar(EV_EMERGENCIA, 1); cambiar_estado(emergencia); }
		}
		else
		{
			bt_enviar(RSP_NACK | ERR_RECHAZADO, reg_cmd);   // solo se sale con los botones
		}
		break;

		default:
		errores_chk++;
		bt_enviar(RSP_NACK | ERR_COMANDO, reg_cmd);
		break;
	}
}


void configurar_interrupciones(void) {
	// Botones INT0 e INT1
	DDRD &= ~((1 << PD2) | (1 << PD3));
	PORTD |= (1 << PD2) | (1 << PD3);
	_delay_ms(50);
	
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
	contador_ms += paso_ms;
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
	if (por_nivel) EIMSK &= ~(1 << INT0);   // por nivel se repetiria mientras este apretado
	evento_boton = 1;
}

ISR(INT1_vect)
{
	if (por_nivel) EIMSK &= ~(1 << INT1);
	evento_emergencia = 1;
}

ISR(WDT_vect)
{
	if (modo_reset) marca_bloqueo = 0xB10C;   // se vencio nuestro WDT: en el proximo vencimiento resetea
	else evento_wdt = 1;
}
