#include "i2c_lcd.h"

#define LCD_I2C_ADDR 0x27 // Change this if your LCD uses a different address

void i2c_init(void) {
    TWSR = 0x00;
    TWBR = 0x0C;  // Set bit rate (100kHz at 16MHz CPU clock)
    TWCR = (1<<TWEN);  // Enable TWI
}

void i2c_start(void) {
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

void i2c_stop(void) {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
    while (TWCR & (1<<TWSTO));
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

void lcd_send_cmd(uint8_t cmd) {
    uint8_t data_u, data_l;
    data_u = (cmd & 0xF0);
    data_l = ((cmd << 4) & 0xF0);
    i2c_start();
    i2c_write(LCD_I2C_ADDR << 1);
    i2c_write(data_u | LCD_BACKLIGHT);
    i2c_write(data_u | LCD_EN | LCD_BACKLIGHT);
    _delay_us(1);
    i2c_write(data_u & ~LCD_EN | LCD_BACKLIGHT);
    _delay_us(200);
    i2c_write(data_l | LCD_BACKLIGHT);
    i2c_write(data_l | LCD_EN | LCD_BACKLIGHT);
    _delay_us(1);
    i2c_write(data_l & ~LCD_EN | LCD_BACKLIGHT);
    _delay_us(2);
    i2c_stop();
}

void lcd_send_data(uint8_t data) {
    uint8_t data_u, data_l;
    data_u = (data & 0xF0);
    data_l = ((data << 4) & 0xF0);
    i2c_start();
    i2c_write(LCD_I2C_ADDR << 1);
    i2c_write(data_u | LCD_RS | LCD_BACKLIGHT);
    i2c_write(data_u | LCD_RS | LCD_EN | LCD_BACKLIGHT);
    _delay_us(1);
    i2c_write(data_u | LCD_RS | LCD_BACKLIGHT);
    _delay_us(200);
    i2c_write(data_l | LCD_RS | LCD_BACKLIGHT);
    i2c_write(data_l | LCD_RS | LCD_EN | LCD_BACKLIGHT);
    _delay_us(1);
    i2c_write(data_l | LCD_RS | LCD_BACKLIGHT);
    _delay_us(2);
    i2c_stop();
}

void lcd_init(void) {
    i2c_init();
    _delay_ms(50);
    lcd_send_cmd(0x30);
    _delay_ms(5);
    lcd_send_cmd(0x30);
    _delay_ms(1);
    lcd_send_cmd(0x30);
    _delay_ms(10);
    lcd_send_cmd(0x20);
    _delay_ms(10);
    lcd_send_cmd(LCD_FUNCTIONSET | LCD_2LINE);
    lcd_send_cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYON);
    lcd_send_cmd(LCD_CLEARDISPLAY);
    _delay_ms(2);
    lcd_send_cmd(LCD_ENTRYMODESET | LCD_ENTRYLEFT);
}

void lcd_clear(void) {
    lcd_send_cmd(LCD_CLEARDISPLAY);
    _delay_ms(2);
}

void lcd_home(void) {
    lcd_send_cmd(LCD_RETURNHOME);
    _delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    lcd_send_cmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}