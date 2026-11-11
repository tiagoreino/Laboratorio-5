#ifndef LCD_CON_I2C_H_
#define LCD_CON_I2C_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <util/delay.h>
#include <avr/io.h>

// ============= || LCD PARA I2C || =================
// LCD 16x2 con modulo I2C
#define LCD_ADDR  0x27   // direccion I2C de 7 bits
#define LCD_RS  0x01
#define LCD_E   0x04
#define LCD_BL  0x08

void twi_init(void);
void twi_start(uint8_t direccion_escritura);
void twi_write(uint8_t dato);
void twi_stop(void);
void lcd_nibble(uint8_t nibble);
void lcd_enviar(uint8_t dato, uint8_t rs);
void lcd_cmd(uint8_t cmd);
void lcd_clear(void);
void lcd_init(void);
void lcd_set_cursor(uint8_t col, uint8_t fila);
void lcd_print(const char *s);
void lcd_titulo(const char *linea1, const char *linea2);
void lcd_power_off(void);   // display off + retroiluminación off
void lcd_power_on(void);    // luz on + re-sincroniza el modo 4 bits

#endif