#ifndef OLED_H
#define OLED_H

#include <stdint.h>
extern const unsigned char F6x8[][6];
extern const unsigned char F8X16[][16];
void oled_init(void);
void oled_set_pos(uint8_t x, uint8_t page);
void oled_write_char(uint8_t x, uint8_t page, char ch, uint8_t size);//ÏÔÊ¾Ò»¸ö×Ö·û
void oled_write_string(uint8_t x, uint8_t page, const char *s, uint8_t size);

#endif /*OLED_H*/
