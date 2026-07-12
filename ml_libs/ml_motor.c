#include "headfile.h"

#define LEFT_MOTOR_REVERSE   0
#define RIGHT_MOTOR_REVERSE  1

uint8_t motorA_dir=1;   //1为正转 0为反转
uint8_t motorB_dir=1;


void motor_init()    //初始化
{
		pwm_init(TIMG8,DL_TIMER_CC_0_INDEX,1000);	     //A通道 右轮 PWMA=PB15
		gpio_init(GPIOA,DL_GPIO_PIN_13,PA13,OUT);      //AIN1
		gpio_init(GPIOA,DL_GPIO_PIN_12,PA12,OUT);      //AIN2
	
	  pwm_init(TIMG8,DL_TIMER_CC_1_INDEX,1000);	    //B通道 左轮 PWMB=PB16
		gpio_init(GPIOB,DL_GPIO_PIN_0,PB0,OUT);       //BIN1
		gpio_init(GPIOB,DL_GPIO_PIN_1,PB1,OUT);       //BIN2
}



void Set_left_pwm(int pwm)
{
#if LEFT_MOTOR_REVERSE
    pwm = -pwm;
#endif

	  if(pwm>0)
	  {
		gpio_set(GPIOB,DL_GPIO_PIN_0,1);
		gpio_set(GPIOB,DL_GPIO_PIN_1,0);
		pwm_update(TIMG8,DL_TIMER_CC_1_INDEX,pwm);
	  }
	  else if(pwm==0)
	  {
		gpio_set(GPIOB,DL_GPIO_PIN_0,0);
		gpio_set(GPIOB,DL_GPIO_PIN_1,0);
		pwm_update(TIMG8,DL_TIMER_CC_1_INDEX,pwm);	
	  }
	  else
	  {
		gpio_set(GPIOB,DL_GPIO_PIN_0,0);
		gpio_set(GPIOB,DL_GPIO_PIN_1,1);
		pwm_update(TIMG8,DL_TIMER_CC_1_INDEX,-pwm);
	  }
}


void Set_right_pwm(int pwm)
{
#if RIGHT_MOTOR_REVERSE
    pwm = -pwm;
#endif

	if(pwm>0)
	{
		gpio_set(GPIOA,DL_GPIO_PIN_13,1);
		gpio_set(GPIOA,DL_GPIO_PIN_12,0);
		pwm_update(TIMG8,DL_TIMER_CC_0_INDEX,pwm);	
	}
	else if(pwm==0)
	{
		gpio_set(GPIOA,DL_GPIO_PIN_13,0);
		gpio_set(GPIOA,DL_GPIO_PIN_12,0);
		pwm_update(TIMG8,DL_TIMER_CC_0_INDEX,pwm);	
	}
	else{
		gpio_set(GPIOA,DL_GPIO_PIN_13,0);
		gpio_set(GPIOA,DL_GPIO_PIN_12,1);
		pwm_update(TIMG8,DL_TIMER_CC_0_INDEX,-pwm);
	}
}
	
	

