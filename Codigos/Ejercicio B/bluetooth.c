#define F_CPU 16000000UL

#include <util/delay.h>
#include "bluetooth.h"

static volatile uint8_t buf[BUF_N];
static volatile uint8_t head = 0, tail = 0;
typedef enum { ESP_INICIO, ESP_DATA, ESP_CHK, ESP_STOP } rx_t;
static rx_t st = ESP_INICIO;
static uint8_t cmd_rx;
static uint16_t t_ultimo = 0;
volatile uint16_t errores_chk = 0;

ISR(USART_RX_vect) {
	uint8_t c = UDR0;
	uint8_t next = (head + 1) & (BUF_N - 1);
	if (next != tail) { buf[head] = c; head = next; }
	// si está lleno, se descarta el byte
}

uint8_t uart_hay(void) { return head != tail; }

static uint8_t uart_leer(void) {
	uint8_t c = buf[tail];
	tail = (tail + 1) & (BUF_N - 1);
	return c;
}

// ============= || BlueTooth HC-05 || =================

void uart_init(void)
{
	UBRR0H = 0;
	UBRR0L = 103;                                     // 9600 baud @ 16 MHz
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);   // RXCIE0: sin esto la ISR nunca corre
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);               // 8N1
}

void uart_putc(char c)
{
	if (PRR & (1<<PRUSART0)) return;
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void bt_enviar(uint8_t tipo, uint8_t dato)
{
	uart_putc(0xBB);
	uart_putc(tipo);
	uart_putc(dato);
	uart_putc(0xBB ^ tipo ^ dato);
	uart_putc(0x55);
}

// Devuelve 1 cuando llega una trama válida y deja el byte en *cmd
uint8_t parser_tarea(uint8_t *cmd, uint16_t ahora_ms) {
	if (ahora_ms < t_ultimo) t_ultimo = ahora_ms;     // contador_ms se pone en 0 con cada actividad
	if (st != ESP_INICIO && (uint16_t)(ahora_ms - t_ultimo) > BT_TIMEOUT_MS) {
		st = ESP_INICIO;
		bt_enviar(RSP_NACK | ERR_INCOMPLETA, 0);
	}
	while (uart_hay()) {
		uint8_t c = uart_leer();
		t_ultimo = ahora_ms;
		switch (st) {
			case ESP_INICIO: if (c == 0xAA) st = ESP_DATA; break;
			case ESP_DATA:   cmd_rx = c; st = ESP_CHK;    break;
			case ESP_CHK:
				if (c == (cmd_rx ^ 0xAA)) {
					st = ESP_STOP;
				} else {
					errores_chk++;
					bt_enviar(RSP_NACK | ERR_CHECKSUM, cmd_rx);
					st = (c == 0xAA) ? ESP_DATA : ESP_INICIO;
				}
				break;
			case ESP_STOP:
				st = ESP_INICIO;
				if (c == 0x55) { *cmd = cmd_rx; return 1; }
				bt_enviar(RSP_NACK | ERR_STOP, cmd_rx);
				if (c == 0xAA) st = ESP_DATA;
				break;
		}
	}
	return 0;
}

void bt_apagar(void)
{
	if (PRR & (1<<PRUSART0)) return;
	while (!(UCSR0A & (1<<UDRE0)));
	_delay_ms(2);                       // que termine de salir el último byte (~1 ms a 9600)
	UCSR0B = 0;
	PORTD |= (1<<PD1);                  // TX queda en alto (reposo)
	DDRD  |= (1<<PD1);
	PRR |= (1<<PRUSART0);
	head = tail = 0;
	st = ESP_INICIO;
}

void bt_prender(void)
{
	if (!(PRR & (1<<PRUSART0))) return;
	PRR &= ~(1<<PRUSART0);
	uart_init();
}
