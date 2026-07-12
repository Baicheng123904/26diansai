#include "headfile.h"

void TIMG0_IRQHandler(void)
{
    if(DL_TimerG_getPendingInterrupt(TIMG0) == DL_TIMER_IIDX_LOAD)
    {
    }
}

void TIMG6_IRQHandler(void)
{
    if(DL_TimerG_getPendingInterrupt(TIMG6) == DL_TIMER_IIDX_LOAD)
    {
    }
}

void TIMG7_IRQHandler(void)
{
    if(DL_TimerG_getPendingInterrupt(TIMG7) == DL_TIMER_IIDX_LOAD)
    {
        DL_TimerG_disablePower(TIMG7);
    }
}

void TIMG12_IRQHandler(void)
{
    if(DL_TimerG_getPendingInterrupt(TIMG12) == DL_TIMER_IIDX_LOAD)
    {
    }
}

void UART0_IRQHandler(void)
{
    if(DL_UART_getPendingInterrupt(UART0) == DL_UART_IIDX_RX)
    {
        imu_uart_callback();
    }
    NVIC_ClearPendingIRQ(UART0_INT_IRQn);
}

void UART1_IRQHandler(void)
{
    if(DL_UART_getPendingInterrupt(UART1) == DL_UART_IIDX_RX)
    {
    }
}

void UART2_IRQHandler(void)
{
    if(DL_UART_getPendingInterrupt(UART2) == DL_UART_IIDX_RX)
    {
    }
}

void UART3_IRQHandler(void)
{
    if(DL_UART_getPendingInterrupt(UART3) == DL_UART_IIDX_RX)
    {
    }
}
