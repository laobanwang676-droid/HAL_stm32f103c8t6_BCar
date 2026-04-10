#ifndef APP_H
#define APP_H

typedef void (* systick_cb)(void);

void app_init(void);
void app_loop(void);

#endif /* APP_H */

