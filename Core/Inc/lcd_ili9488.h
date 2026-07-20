#ifndef LCD_ILI9488_H
#define LCD_ILI9488_H

#include "main.h"

#define LCD_WIDTH  320
#define LCD_HEIGHT 480

#define LCD_RGB(r, g, b) ((uint16_t)(((r)&0xF8)<<8) | (((g)&0xFC)<<3) | ((b)>>3))

#define LCD_BLACK   0x0000
#define LCD_WHITE   0xFFFF
#define LCD_RED     0xF800
#define LCD_GREEN   0x07E0
#define LCD_BLUE    0x001F
#define LCD_YELLOW  0xFFE0
#define LCD_CYAN    0x07FF
#define LCD_MAGENTA 0xF81F
#define LCD_GRAY    0x8410

void LCD_Init(void);
void LCD_Clear(uint16_t color);
void LCD_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_Print(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg);

#endif
