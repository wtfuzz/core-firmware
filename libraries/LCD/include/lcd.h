#ifndef SPARK_LCD_H_
#define SPARK_LCD_H_

#include <lcd_font.h>
#include <lcd_ssd1306.h>

typedef enum _lcd_type_t
{
  LCD_TYPE_SSD1306,
} lcd_type_t;

typedef struct _lcd_t
{
  /** Pins */
  int reset;
  int cs;
  int dc;

  // Character column
  uint8_t x;
  // Character row
  uint8_t y;
} lcd_t;

void ssd1306_init(lcd_t *lcd, int reset_pin, int cs_pin, int dc_pin);
void ssd1306_write(lcd_t *lcd, char c);
void ssd1306_print(lcd_t *lcd, char *str);
void ssd1306_cursor(lcd_t *lcd, int x, int y);
void ssd1306_clear(lcd_t *lcd);

#endif
