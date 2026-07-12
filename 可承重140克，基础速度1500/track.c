#include "headfile.h"

void track_init(void)
{
    xunji_init();
}

int get_line_color(void)
{
    if(D1 || D2 || D3 || D4 || D5 || D6 || D7 || D8)
    {
        return 1;
    }

    return 0;
}
