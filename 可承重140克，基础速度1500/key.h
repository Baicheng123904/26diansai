#ifndef __KEY_H__
#define __KEY_H__

extern volatile int mode;
extern volatile int set;

void key_init();
uint8_t key_GetNum(void);

#endif

