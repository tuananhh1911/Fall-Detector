#include "stm32f10x.h"
#include "LCD.h"

// I2C address of the PCF8574
#define LCD_ADDR (0x27 << 1)

// LCD commands
#define LCD_CMD_CLEAR_DISPLAY 0x01
#define LCD_CMD_RETURN_HOME 0x02
#define LCD_CMD_ENTRY_MODE_SET 0x04
#define LCD_CMD_DISPLAY_CONTROL 0x08
#define LCD_CMD_CURSOR_SHIFT 0x10
#define LCD_CMD_FUNCTION_SET 0x20
#define LCD_CMD_SET_CGRAM_ADDR 0x40
#define LCD_CMD_SET_DDRAM_ADDR 0x80

// LCD flags
#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00

#define LCD_DISPLAY_MOVE 0x08
#define LCD_CURSOR_MOVE 0x00
#define LCD_MOVE_RIGHT 0x04
#define LCD_MOVE_LEFT 0x00

#define LCD_8BIT_MODE 0x10
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// Backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0x04  // Enable bit
#define Rw 0x02  // Read/Write bit
#define Rs 0x01  // Register select bit

static void lcd_i2c_write(uint8_t data);
static void lcd_i2c_write_cmd(uint8_t cmd);
static void lcd_i2c_write_data(uint8_t data);
static void lcd_i2c_toggle_enable(uint8_t data);

void lcd_init(void)
{
    // Wait for the LCD to power up
    for (volatile int i = 0; i < 50000; i++);

    // Initialize the LCD in 4-bit mode
    lcd_i2c_write(0x30);
    lcd_i2c_toggle_enable(0x30);
    for (volatile int i = 0; i < 5000; i++);
    lcd_i2c_toggle_enable(0x30);
    for (volatile int i = 0; i < 200; i++);
    lcd_i2c_toggle_enable(0x30);

    lcd_i2c_toggle_enable(0x20);

    // Function set: 4-bit mode, 2 lines, 5x8 dots
    lcd_i2c_write_cmd(LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8DOTS);

    // Display control: display on, cursor off, no blink
    lcd_i2c_write_cmd(LCD_CMD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);

    // Clear display
    lcd_i2c_write_cmd(LCD_CMD_CLEAR_DISPLAY);

    // Entry mode set: increment automatically, no display shift
    lcd_i2c_write_cmd(LCD_CMD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);

    // Return home
    lcd_i2c_write_cmd(LCD_CMD_RETURN_HOME);
}

void lcd_clear(void)
{
    lcd_i2c_write_cmd(LCD_CMD_CLEAR_DISPLAY);
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    static uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    lcd_i2c_write_cmd(LCD_CMD_SET_DDRAM_ADDR | (col + row_offsets[row]));
}

void lcd_print(char *str)
{
    while (*str)
    {
        lcd_i2c_write_data((uint8_t)(*str));
        str++;
    }
}

static void lcd_i2c_write(uint8_t data)
{
    while (I2C1->SR2 & I2C_SR2_BUSY);  // Wait until I2C1 is not busy anymore

    I2C1->CR1 |= I2C_CR1_START;  // Generate start condition
    while (!(I2C1->SR1 & I2C_SR1_SB));  // Wait until start condition is generated

    I2C1->DR = LCD_ADDR;  // Send I2C address
    while (!(I2C1->SR1 & I2C_SR1_ADDR));  // Wait until address is sent
    (void)I2C1->SR2;  // Clear address flag

    I2C1->DR = data | LCD_BACKLIGHT;  // Send data with backlight on
    while (!(I2C1->SR1 & I2C_SR1_BTF));  // Wait until data is transferred

    I2C1->CR1 |= I2C_CR1_STOP;  // Generate stop condition
}

static void lcd_i2c_write_cmd(uint8_t cmd)
{
    lcd_i2c_write((cmd & 0xF0) | En | LCD_BACKLIGHT);
    lcd_i2c_toggle_enable((cmd & 0xF0) | LCD_BACKLIGHT);
    lcd_i2c_write(((cmd << 4) & 0xF0) | En | LCD_BACKLIGHT);
    lcd_i2c_toggle_enable(((cmd << 4) & 0xF0) | LCD_BACKLIGHT);
}

static void lcd_i2c_write_data(uint8_t data)
{
    lcd_i2c_write((data & 0xF0) | Rs | En | LCD_BACKLIGHT);
    lcd_i2c_toggle_enable((data & 0xF0) | Rs | LCD_BACKLIGHT);
    lcd_i2c_write(((data << 4) & 0xF0) | Rs | En | LCD_BACKLIGHT);
    lcd_i2c_toggle_enable(((data << 4) & 0xF0) | Rs | LCD_BACKLIGHT);
}

static void lcd_i2c_toggle_enable(uint8_t data)
{
    lcd_i2c_write(data | En);
    for (volatile int i = 0; i < 500; i++);  // Short delay
    lcd_i2c_write(data & ~En);
    for (volatile int i = 0; i < 500; i++);  // Short delay
}