#ifndef LCD_CON_I2C_H_    
#define LCD_CON_I2C_H_       

#include <avr/io.h>

// ============= || LCD PARA I2C || =================
// LCD 16x2 con modulo I2C
#define LCD_ADDR  0x27   // direccion I2C de 7 bits
#define LCD_RS  0x01
#define LCD_E   0x04
#define LCD_BL  0x08

//=========== Inicialización modo maestro =============
static void twi_init(void);

// ============== Habilita protocolo TWI ============
static void twi_start(uint8_t direccion_escritura);

// ========== Enviar dato por TWI ==============
static void twi_write(uint8_t dato);

// =========== Detener Envio de Datos TWI ===========
static void twi_stop(void);

// ========== Definición de nibble =============
static void lcd_nibble(uint8_t nibble);

// ========== Enviar byte en dos nibbles ==============
static void lcd_enviar(uint8_t dato, uint8_t rs);

// ========== Enviar Comando directo =================
static void lcd_cmd(uint8_t cmd);

// ============ Enviar Comando de limpiar pantalla 0x01 =========
void lcd_clear(void);

// ========== Inicialización de LCD =============
void lcd_init(void);

// ========== Posicionar cursor ==============
void lcd_set_cursor(uint8_t col, uint8_t fila);

// ========== Enviar Caracter ================
void lcd_print(const char *s);

// ========= Borra y escribe las dos lineas ==============
void lcd_titulo(const char *linea1, const char *linea2);

#endif 