#ifndef __ENCODER_H__
#define __ENCODER_H__

extern int left_encoder;
extern int right_encoder;
extern volatile int mode;
extern volatile int set;

void encoder_init(void);
void encoder_clear_counts(void);
int read_encoder1(void);
int read_encoder2(void);


#endif


