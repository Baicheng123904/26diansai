#include "headfile.h"

volatile int Encoder_count1 = 0;
volatile int Encoder_count2 = 0;
volatile int mode,set;

void encoder_init(void)
{
    exti_init(EXTI_PA17, RISING, 0);   // 左轮 A
    exti_init(EXTI_PA24, FALLING, 0);  // 左轮 B
    gpio_init(GPIOA, DL_GPIO_PIN_17, PA17, IN_UP);
    gpio_init(GPIOA, DL_GPIO_PIN_24, PA24, IN_DOWN);

    exti_init(EXTI_PA15, RISING, 0);   // 右轮 A
    exti_init(EXTI_PA16, FALLING, 0);  // 右轮 B
    gpio_init(GPIOA, DL_GPIO_PIN_15, PA15, IN_UP);
    gpio_init(GPIOA, DL_GPIO_PIN_16, PA16, IN_DOWN);
}

void encoder_clear_counts(void)
{
    __disable_irq();
    Encoder_count1 = 0;
    Encoder_count2 = 0;
    __enable_irq();
}

int read_encoder1(void)
{
    int num;
    __disable_irq();
    num = Encoder_count1;
    Encoder_count1 = 0;
    __enable_irq();
    return num;
}

int read_encoder2(void)
{
    int num;
    __disable_irq();
    num = Encoder_count2;
    Encoder_count2 = 0;
    __enable_irq();
    return num;
}

void GROUP1_IRQHandler(void)
{
    if(GPIOA->CPU_INT.MIS & DL_GPIO_PIN_17)
    {
        if(gpio_get(GPIOA, DL_GPIO_PIN_24) == 0) Encoder_count1++;
        else Encoder_count1--;
        DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_17);
    }

    if(GPIOA->CPU_INT.MIS & DL_GPIO_PIN_24)
    {
        if(gpio_get(GPIOA, DL_GPIO_PIN_17) == 0) Encoder_count1++;
        else Encoder_count1--;
        DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_24);
    }

    if(GPIOA->CPU_INT.MIS & DL_GPIO_PIN_15)
    {
        if(gpio_get(GPIOA, DL_GPIO_PIN_16) == 0) Encoder_count2++;
        else Encoder_count2--;
        DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_15);
    }

    if(GPIOA->CPU_INT.MIS & DL_GPIO_PIN_16)
    {
        if(gpio_get(GPIOA, DL_GPIO_PIN_15) == 0) Encoder_count2++;
        else Encoder_count2--;
        DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_16);
    }
}
