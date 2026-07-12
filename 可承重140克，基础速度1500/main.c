#include "headfile.h"

uint8_t KeyNum;
int left_pwm,right_pwm,left_angle_pwm,right_angle_pwm;
int base_left_pwm,base_right_pwm,angle_pwm,angle_pwm1;
int cur_state,last_state,pre_state;
unsigned int Temp[2] = {0};
int turn_pwm,cnt;

static void stop_car(void)
{
    Set_left_pwm(0);
    Set_right_pwm(0);
}

static void reset_run_state(void)
{
    stop_car();
    pid_reset_all();
    encoder_clear_counts();
    xunji_runtime_reset();
    left_encoder = 0;
    right_encoder = 0;
}

static void control_tick(void)
{
    int encoder1 = read_encoder1();
    int encoder2 = read_encoder2();

    /* 后退循迹时，轮子编码器的速度符号相对前进运行反向。 */
    if(xunji_drive_reverse_enabled())
    {
        left_encoder = encoder1;
        right_encoder = encoder2;
    }
    else
    {
        left_encoder = -encoder1;
        right_encoder = -encoder2;
    }

    if(set == 0 || mode == 0)
    {
        stop_car();
        return;
    }

    if(mode == 1)
    {
        track1();
    }
    else if(mode == 2)
    {
        track2();
    }
    else if(mode == 3)
    {
        track3();
    }
    else
    {
        track4();
    }
}

static void show_status(void)
{
    OLED_ShowString(1, 1, "m:");
    OLED_ShowString(1, 5, "s:");
    OLED_ShowString(2, 1, "yaw:");
    OLED_ShowString(3, 1, "raw:");
    OLED_ShowString(4, 1, "sen:");
    OLED_ShowNum(1, 3, mode, 1);
    OLED_ShowNum(1, 7, set, 1);
    OLED_ShowSignedNum(2, 6, yaw_angle_int, 4);
    OLED_ShowBinNum(3, 6, xunji_raw_bits(), 8);
    OLED_ShowBinNum(4, 6, xunji_line_bits(), 8);
}

int main(void)
{
    uint8_t oled_cnt = 0;

    system_init();
    delay_ms(500);

    OLED_Init();
    imu_init();
    key_init();
    motor_init();
    encoder_init();
    xunji_init();

    pid_init(&motorA, DELTA_PID, 6, 3, 1);
    pid_init(&motorB, DELTA_PID, 6, 3, 1);
    pid_init(&angle, POSITION_PID, 0.035f, 0.0f, 0.060f);

    OLED_Clear();
    OLED_ShowString(1, 1, "TMX TASK");

    while(1)
    {
        if(imu_flag)
        {
            imu_flag = 0;
            imu_analysis();
        }

        KeyNum = key_GetNum();
        if(KeyNum == 1)
        {
            mode++;
            if(mode >= 5) mode = 1;
            set = 0;
            reset_run_state();
        }
        else if(KeyNum == 2)
        {
            set = !set;
            reset_run_state();
        }

        control_tick();

        if(++oled_cnt >= 5)
        {
            oled_cnt = 0;
            show_status();
        }

        delay_ms(10);
    }
}
