#ifndef _motor_h
#define _motor_h
#include "headfile.h"

/*                                    
   天猛星 MSPM0G3507 + TB6612FNG:
   A通道为右轮: PWMA=PB15/TIMG8_C0, AIN1=PA13, AIN2=PA12
   B通道为左轮: PWMB=PB16/TIMG8_C1, BIN1=PB0,  BIN2=PB1
*/


void motor_init(void);
void Set_left_pwm(int pwm);
void Set_right_pwm(int pwm);

extern uint8_t motorA_dir, motorB_dir;

#endif


