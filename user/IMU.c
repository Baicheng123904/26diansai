#include "headfile.h"

volatile uint8_t imu_flag;
double yaw_angle;
volatile int yaw_angle_int;
struct Angle YawAngle;

#define IMU_FRAME_LENGTH 11U
#define IMU_FRAME_HEADER 0x55U
#define IMU_FRAME_ANGLE  0x53U

void imu_init(void)
{
    uart_init(UART0, 9600, 1);
}

void imu_uart_callback(void)
{
    static uint8_t imu_rx_buffer[IMU_FRAME_LENGTH];
    static uint8_t counter = 0;
    uint8_t data = uart_getbyte(UART0);
    uint8_t checksum = 0;
    uint8_t i;

    if(counter == 0)
    {
        if(data == IMU_FRAME_HEADER)
        {
            imu_rx_buffer[counter++] = data;
        }
        return;
    }

    imu_rx_buffer[counter++] = data;
    if(counter < IMU_FRAME_LENGTH)
    {
        return;
    }

    for(i = 0; i < (IMU_FRAME_LENGTH - 1U); i++)
    {
        checksum += imu_rx_buffer[i];
    }

    if((checksum == imu_rx_buffer[IMU_FRAME_LENGTH - 1U]) &&
       (imu_rx_buffer[1] == IMU_FRAME_ANGLE))
    {
        memcpy(&YawAngle, &imu_rx_buffer[2], sizeof(YawAngle));
        imu_flag = 1;
        counter = 0;
        return;
    }

    /* Keep the next frame header after a failed checksum to resynchronize. */
    for(i = 1; i < IMU_FRAME_LENGTH; i++)
    {
        if(imu_rx_buffer[i] == IMU_FRAME_HEADER)
        {
            uint8_t remaining = IMU_FRAME_LENGTH - i;
            uint8_t j;

            for(j = 0; j < remaining; j++)
            {
                imu_rx_buffer[j] = imu_rx_buffer[i + j];
            }
            counter = remaining;
            return;
        }
    }

    counter = 0;
}

void imu_analysis(void)
{
    struct Angle sample;

    __disable_irq();
    sample = YawAngle;
    __enable_irq();

    yaw_angle = (float)sample.Angle[2] / 32768 * 180;
    yaw_angle_int = (int)yaw_angle;
}
