#include "headfile.h"

void key_init(void)
{
    set = 0;
    mode = 0;
    gpio_init(GPIOA, DL_GPIO_PIN_30, PA30, IN_UP);
    gpio_init(GPIOA, DL_GPIO_PIN_18, PA18, IN_UP);
}

uint8_t key_GetNum(void)
{
    uint8_t KeyNum = 0;
    static uint8_t key1_last = 1;
    static uint8_t key2_last = 1;
    static uint8_t debounce_ticks = 0;
    uint8_t key1_now = gpio_get(GPIOA, DL_GPIO_PIN_30) ? 1 : 0;
    uint8_t key2_now = gpio_get(GPIOA, DL_GPIO_PIN_18) ? 1 : 0;

    if(debounce_ticks > 0)
    {
        debounce_ticks--;
        key1_last = key1_now;
        key2_last = key2_now;
        return 0;
    }

    if(key1_last == 1 && key1_now == 0)
    {
        KeyNum = 1;
        debounce_ticks = 3;
    }

    if(KeyNum == 0 && key2_last == 1 && key2_now == 0)
    {
        KeyNum = 2;
        debounce_ticks = 3;
    }

    key1_last = key1_now;
    key2_last = key2_now;

    return KeyNum;
}
