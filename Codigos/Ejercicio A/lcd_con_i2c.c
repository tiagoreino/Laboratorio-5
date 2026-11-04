
#define F_CPU 16000000UL

#include "lcd_con_i2c.h"

// ============= || LCD PARA I2C || =================

//=========== Inicialización modo maestro =============
void twi_init(void) {
	TWSR = 0;        // prescaler del TWI = 1
	TWBR = 72;       // SCL = F_CPU / (16 + 2*TWBR) = 16 MHz / 160 = 100 kHz
}

// ============== Habilita protocolo TWI ============
void twi_start(uint8_t direccion_escritura) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);   // condicion START
	while (!(TWCR & (1 << TWINT)));
	TWDR = direccion_escritura;                          // direccion + bit W
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

// ========== Enviar dato por TWI ==============
void twi_write(uint8_t dato) {
	TWDR = dato;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

// =========== Detener Envio de Datos TWI ===========
void twi_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);   // condicion STOP
}

// ========== Definición de nibble (CORREGIDO) =============
void lcd_nibble(uint8_t nibble) {
	uint8_t d = ((nibble << 4) & 0xF0) | LCD_BL;
	twi_start(LCD_ADDR << 1);
	twi_write(d | LCD_E);    // E = 1
	_delay_us(2);            // Pulso mínimo de Enable (Data setup time)
	twi_write(d);            // E = 0 (el LCD lee el dato en este flanco de bajada)
	_delay_us(40);           // Tiempo para que el LCD procese el nibble
	twi_stop();
}

// ========== Enviar byte en dos nibbles ==============
void lcd_enviar(uint8_t dato, uint8_t rs) {
	uint8_t alto = (dato & 0xF0)        | LCD_BL | rs;
	uint8_t bajo = ((dato << 4) & 0xF0) | LCD_BL | rs;
	twi_start(LCD_ADDR << 1);
	
	twi_write(alto | LCD_E);
	_delay_us(2);
	twi_write(alto);
	_delay_us(40);
	
	twi_write(bajo | LCD_E);
	_delay_us(2);
	twi_write(bajo);
	_delay_us(40);
	
	twi_stop();
}

// ========== Enviar Comando directo =================
void lcd_cmd(uint8_t cmd) {
	lcd_enviar(cmd, 0);
}

// ============ Enviar Comando de limpiar pantalla =========
void lcd_clear(void) {
	lcd_cmd(0x01);
	_delay_ms(2);            // "clear" es lento (~1,5 ms)
}

// ========== Inicialización de LCD =============
void lcd_init(void) {
	twi_init();
	_delay_ms(50);           // el LCD necesita tiempo para arrancar

	// Secuencia de arranque estricta del HD44780
	lcd_nibble(0x03);  _delay_ms(5);       // Espera > 4.1ms
	lcd_nibble(0x03);  _delay_us(150);     // Espera > 100us
	lcd_nibble(0x03);  _delay_us(150);
	lcd_nibble(0x02);  _delay_ms(2);       // Configura modo 4 bits de forma segura

	lcd_cmd(0x28);           // 4 bits, 2 lineas, fuente 5x8
	lcd_cmd(0x0C);           // display encendido, sin cursor
	lcd_cmd(0x06);           // el cursor avanza hacia la derecha
	lcd_clear();
}

// ========== Posicionar cursor ==============
void lcd_set_cursor(uint8_t col, uint8_t fila) {
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

// Apaga por completo el display y la luz (Modo Seguro)
void lcd_power_off(void) {
	lcd_cmd(0x08); // Comando HD44780: Display OFF, Cursor OFF, Blink OFF
	
	// Apagamos la luz de fondo dejando el expansor I2C en un estado limpio
	twi_start(LCD_ADDR << 1);
	twi_write(0x00);
	twi_stop();
}

// Enciende por completo el display, la luz y RESINCRONIZA los nibbles
void lcd_power_on(void) {
	// 1. Primero encendemos la luz de fondo de manera limpia
	twi_start(LCD_ADDR << 1);
	twi_write(LCD_BL);
	twi_stop();
	_delay_ms(2); // Pequeña pausa para estabilizar tensiones en el expansor
	
	// 2. FORZAR RE-ALINEACIÓN DE 4 BITS (Secuencia mágica de resincronización)
	// Esto vacía cualquier medio-nibble que haya quedado colgado por ruido eléctrico.
	lcd_nibble(0x03);  _delay_us(150);
	lcd_nibble(0x03);  _delay_us(150);
	lcd_nibble(0x02);  _delay_ms(2);   // Asegura volver al modo 4 bits limpio
	
	// 3. Volver a configurar los parámetros nativos de la pantalla
	lcd_cmd(0x28);     // Modo 4 bits, 2 líneas, fuente 5x8
	lcd_cmd(0x0C);     // Display ON, Cursor OFF
	lcd_clear();       // Limpia la pantalla para borrar cualquier basura previa
}