#include "lcd_con_i2c.h"

// ============= || LCD PARA I2C || =================


//=========== Inicialización modo maestro =============
static void twi_init(void) {
	TWSR = 0;        // prescaler del TWI = 1
	TWBR = 72;       // SCL = F_CPU / (16 + 2*TWBR) = 16 MHz / 160 = 100 kHz
}

// ============== Habilita protocolo TWI ============
static void twi_start(uint8_t direccion_escritura) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);   // condicion START
	while (!(TWCR & (1 << TWINT)));
	TWDR = direccion_escritura;                          // direccion + bit W
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

// ========== Enviar dato por TWI ==============
static void twi_write(uint8_t dato) {
	TWDR = dato;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

// =========== Detener Envio de Datos TWI ===========
static void twi_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);   // condicion STOP
}

// ========== Definición de nibble =============
static void lcd_nibble(uint8_t nibble) {
	uint8_t d = ((nibble << 4) & 0xF0) | LCD_BL;
	twi_start(LCD_ADDR << 1);
	twi_write(d | LCD_E);    // E = 1
	twi_write(d);            // E = 0 (el LCD lee el dato en este flanco de bajada)
	twi_stop();
}

// ========== Enviar byte en dos nibbles ==============
// Manda un byte completo: dos nibbles, cada uno con su pulso de E.
// rs = 0 -> comando, rs = LCD_RS -> caracter. Todo en UNA sola transaccion I2C.
static void lcd_enviar(uint8_t dato, uint8_t rs) {
	uint8_t alto = (dato & 0xF0)        | LCD_BL | rs;
	uint8_t bajo = ((dato << 4) & 0xF0) | LCD_BL | rs;
	twi_start(LCD_ADDR << 1);
	twi_write(alto | LCD_E);
	twi_write(alto);
	twi_write(bajo | LCD_E);
	twi_write(bajo);
	twi_stop();
}

// ========== Enviar Comando directo =================
static void lcd_cmd(uint8_t cmd) {
	lcd_enviar(cmd, 0);
}

// ============ Enviar Comando de limpiar pantalla 0x01 =========
void lcd_clear(void) {
	lcd_cmd(0x01);
	_delay_ms(2);            // "clear" es lento (~1,5 ms)
}

// ========== Inicialización de LCD =============
void lcd_init(void) {
	twi_init();
	_delay_ms(50);           // el LCD necesita tiempo para arrancar

	// Secuencia de arranque del HD44780 para forzar el modo de 4 bits
	lcd_nibble(0x03);  _delay_ms(5);
	lcd_nibble(0x03);  _delay_us(150);
	lcd_nibble(0x03);  _delay_us(150);
	lcd_nibble(0x02);  _delay_us(150);   // pasa a 4 bits

	lcd_cmd(0x28);           // 4 bits, 2 lineas, fuente 5x8
	lcd_cmd(0x0C);           // display encendido, sin cursor
	lcd_cmd(0x06);           // el cursor avanza hacia la derecha
	lcd_clear();
}

// ========== Posicionar cursor ==============
void lcd_set_cursor(uint8_t col, uint8_t fila) {
	// La linea 1 empieza en la direccion 0x00 y la linea 2 en 0x40
	lcd_cmd(0x80 | ((fila ? 0x40 : 0x00) + col));
}

// ========== Enviar Caracter ================
void lcd_print(const char *s) {
	while (*s) {
		lcd_enviar((uint8_t)*s++, LCD_RS);
	}
}

// ========= Borra y escribe las dos lineas ==============
void lcd_titulo(const char *linea1, const char *linea2) {
	lcd_clear();
	lcd_set_cursor(0, 0);
	lcd_print(linea1);
	lcd_set_cursor(0, 1);
	lcd_print(linea2);
}