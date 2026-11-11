#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include <avr/io.h>
#include <avr/interrupt.h>

// ============= || BT PARA EJERCICIO B || =================
// Celular -> micro: [0xAA][CMD|DATO][CHK][0x55]        CHK = 0xAA ^ byte
// Micro -> celular: [0xBB][TIPO|SUB][DATO][CHK][0x55]  CHK = 0xBB ^ tipo ^ dato

#define BUF_N 32                      // potencia de 2
#define BT_TIMEOUT_MS 100

// Respuestas (nibble alto del byte TIPO)
#define RSP_ACK      0xA0             // | comando,  dato = estado
#define RSP_NACK     0xB0             // | error,    dato = byte de comando
#define RSP_DATO     0xC0             // | DATO_xx,  dato = valor
#define RSP_EVENTO   0xD0             // | EV_xx,    dato
#define RSP_REGISTRO 0xE0             // | EV_xx,    dato

#define ERR_CHECKSUM  1
#define ERR_COMANDO   2
#define ERR_DATO      3
#define ERR_RECHAZADO 4
#define ERR_INCOMPLETA 5
#define ERR_STOP      6

#define DATO_ESTADO  0
#define DATO_ADC     1
#define DATO_TEMP    2
#define DATO_PRR     3
#define DATO_WDT     4

#define EV_CAMBIO_ESTADO 1
#define EV_BOTON         2
#define EV_EMERGENCIA    3
#define EV_FIN_EMERG     4
#define EV_RESET         5
#define EV_WDT           6
#define EV_ERROR_DHT     7

extern volatile uint16_t errores_chk;

void uart_init(void);
void uart_putc(char c);
uint8_t uart_hay(void);
void bt_enviar(uint8_t tipo, uint8_t dato);
uint8_t parser_tarea(uint8_t *cmd, uint16_t ahora_ms);
void bt_apagar(void);
void bt_prender(void);

#endif
