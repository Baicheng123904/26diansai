#include "headfile.h"

#define LINE_BASE_PWM      1500
#define LINE_KP            28
#define LINE_KD            4
#define LINE_PWM_LIMIT     2600
#define LINE_CROSS_COUNT   5
#define TURN_BASE_SPEED    22
#define LINE_TURN_SLOW_ERROR   20
#define LINE_TURN_BASE_PWM     1100
#define LINE_SHARP_ERROR       30
#define LINE_SHARP_PWM         1750
#define LINE_SHARP_LOST_PWM    2050
#define LINE_SHARP_BRAKE_TICKS 2
#define LINE_SHARP_EXIT_TICKS  2
#define LINE_CROSS_CONFIRM_TICKS 3
#define LINE_CROSS_RELEASE_TICKS 3
#define LINE_SEARCH_OUTER_PWM  1200
#define LINE_SEARCH_INNER_PWM  350
#define LINE_RAMP_STEP      100
#define LINE_DIFF_LIMIT     8
#define LINE_ERROR_FILTER_NUM  3
#define LINE_BLACK_GPIO_LEVEL  0
/* Rear-facing sensor: D1~D8 are mirrored in the vehicle coordinate frame. */
#define LINE_SENSOR_AT_REAR    1
#define LINE_STEER_REVERSE     1
#define LINE_DRIVE_REVERSE     1

int now_statue,last_statue,change_flag1;
static int s_line_last_error = 0;
static uint8_t s_line_has_last = 0;
static int s_line_left_pwm = 0;
static int s_line_right_pwm = 0;
static int s_line_last_valid_error = 0;
static int s_line_sharp_state = 0;
static uint8_t s_line_sharp_brake_ticks = 0;
static uint8_t s_line_sharp_exit_ticks = 0;
static uint8_t s_line_derivative_valid = 0;
static uint8_t s_line_cross_high_ticks = 0;
static uint8_t s_line_cross_low_ticks = 0;
static uint8_t s_line_cross_latched = 0;
static int s_line_filtered_error = 0;
static uint8_t s_line_filter_valid = 0;

uint8_t xunji_raw_bits(void)
{
    return (gpio_get(GPIOB, DL_GPIO_PIN_25) << 7) |
           (gpio_get(GPIOB, DL_GPIO_PIN_24) << 6) |
           (gpio_get(GPIOB, DL_GPIO_PIN_20) << 5) |
           (gpio_get(GPIOA, DL_GPIO_PIN_14) << 4) |
           (gpio_get(GPIOB, DL_GPIO_PIN_18) << 3) |
           (gpio_get(GPIOB, DL_GPIO_PIN_19) << 2) |
           (gpio_get(GPIOB, DL_GPIO_PIN_10) << 1) |
           gpio_get(GPIOA, DL_GPIO_PIN_7);
}

static uint8_t reverse_bits8(uint8_t value)
{
    value = (uint8_t)(((value & 0x55U) << 1) | ((value >> 1) & 0x55U));
    value = (uint8_t)(((value & 0x33U) << 2) | ((value >> 2) & 0x33U));
    return (uint8_t)((value << 4) | (value >> 4));
}

uint8_t xunji_line_bits(void)
{
    uint8_t raw_bits = xunji_raw_bits();

#if LINE_SENSOR_AT_REAR
    raw_bits = reverse_bits8(raw_bits);
#endif

#if LINE_BLACK_GPIO_LEVEL == 0
    return (uint8_t)(~raw_bits);
#else
    return raw_bits;
#endif
}

void xunji_init(void)
{
    gpio_init(GPIOB, DL_GPIO_PIN_25, PB25, IN_UP);  // D1/L1
    gpio_init(GPIOB, DL_GPIO_PIN_24, PB24, IN_UP);  // D2/L2
    gpio_init(GPIOB, DL_GPIO_PIN_20, PB20, IN_UP);  // D3/L3
    gpio_init(GPIOA, DL_GPIO_PIN_14, PA14, IN_UP);  // D4/L4
    gpio_init(GPIOB, DL_GPIO_PIN_18, PB18, IN_UP);  // D5/R4
    gpio_init(GPIOB, DL_GPIO_PIN_19, PB19, IN_UP);  // D6/R3
    gpio_init(GPIOB, DL_GPIO_PIN_10, PB10, IN_UP);  // D7/R2
    gpio_init(GPIOA, DL_GPIO_PIN_7,  PA7,  IN_UP);  // D8/R1
}

void xunji_runtime_reset(void)
{
    now_statue = 0;
    last_statue = 0;
    change_flag1 = 0;
    s_line_last_error = 0;
    s_line_has_last = 0;
    s_line_left_pwm = 0;
    s_line_right_pwm = 0;
    s_line_last_valid_error = 0;
    s_line_sharp_state = 0;
    s_line_sharp_brake_ticks = 0;
    s_line_sharp_exit_ticks = 0;
    s_line_derivative_valid = 0;
    s_line_cross_high_ticks = 0;
    s_line_cross_low_ticks = 0;
    s_line_cross_latched = 0;
    s_line_filtered_error = 0;
    s_line_filter_valid = 0;
}

unsigned char digtal(unsigned char channel)
{
    uint8_t bits = xunji_line_bits();

    if(channel < 1 || channel > 8)
    {
        return 0;
    }

    return (bits >> (8 - channel)) & 0x01;
}

static int clamp_int(int value, int min, int max)
{
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

static int abs_int(int value)
{
    return (value < 0) ? -value : value;
}

static uint8_t sensor_count_bits(uint8_t bits)
{
    uint8_t count = 0;

    while(bits)
    {
        count += bits & 0x01;
        bits >>= 1;
    }

    return count;
}

static uint8_t line_center_seen_bits(uint8_t bits)
{
    return (bits & 0x3C) ? 1 : 0;
}

static int ramp_to_target(int now, int target)
{
    if(now < target - LINE_RAMP_STEP) return now + LINE_RAMP_STEP;
    if(now > target + LINE_RAMP_STEP) return now - LINE_RAMP_STEP;
    return target;
}

static int line_drive_speed(int speed)
{
#if LINE_DRIVE_REVERSE
    return -speed;
#else
    return speed;
#endif
}

uint8_t xunji_drive_reverse_enabled(void)
{
#if LINE_DRIVE_REVERSE
    return 1;
#else
    return 0;
#endif
}

static void line_set_pwm(int left_target, int right_target)
{
#if LINE_STEER_REVERSE
    int temp = left_target;
    left_target = right_target;
    right_target = temp;
#endif

    left_target = clamp_int(left_target, 0, LINE_PWM_LIMIT);
    right_target = clamp_int(right_target, 0, LINE_PWM_LIMIT);
    s_line_left_pwm = ramp_to_target(s_line_left_pwm, left_target);
    s_line_right_pwm = ramp_to_target(s_line_right_pwm, right_target);
    Set_left_pwm(line_drive_speed(s_line_left_pwm));
    Set_right_pwm(line_drive_speed(s_line_right_pwm));
}

static void line_brake_control(void)
{
    s_line_left_pwm = 0;
    s_line_right_pwm = 0;
    Set_left_pwm(0);
    Set_right_pwm(0);
}

static void line_sharp_turn_control(int pwm)
{
    if(s_line_sharp_state < 0)
    {
        line_set_pwm(0, pwm);
    }
    else if(s_line_sharp_state > 0)
    {
        line_set_pwm(pwm, 0);
    }
}

static void line_search_control(void)
{
    if(s_line_has_last == 0)
    {
        line_set_pwm(LINE_SEARCH_OUTER_PWM, LINE_SEARCH_OUTER_PWM);
    }
    else if(s_line_sharp_state < 0 || s_line_last_error <= -LINE_SHARP_ERROR)
    {
        line_set_pwm(0, LINE_SHARP_LOST_PWM);
    }
    else if(s_line_sharp_state > 0 || s_line_last_error >= LINE_SHARP_ERROR)
    {
        line_set_pwm(LINE_SHARP_LOST_PWM, 0);
    }
    else if(s_line_last_error < 0)
    {
        line_set_pwm(LINE_SEARCH_INNER_PWM, LINE_SEARCH_OUTER_PWM);
    }
    else
    {
        line_set_pwm(LINE_SEARCH_OUTER_PWM, LINE_SEARCH_INNER_PWM);
    }
}

static int line_error_bits(uint8_t bits)
{
    const int weights[8] = {35, 25, 15, 5, -5, -15, -25, -35};
    int sum = 0;
    int count = 0;
    uint8_t i;

    for(i = 0; i < 8; i++)
    {
        if(bits & (0x80 >> i))
        {
            sum += weights[i];
            count++;
        }
    }

    if(count == 0) return 0;
    return sum / count;
}

static int line_filter_error(int raw_error)
{
    if(s_line_filter_valid == 0)
    {
        s_line_filtered_error = raw_error;
        s_line_filter_valid = 1;
    }
    else
    {
        s_line_filtered_error =
            (s_line_filtered_error * (LINE_ERROR_FILTER_NUM - 1) + raw_error) /
            LINE_ERROR_FILTER_NUM;
    }

    return s_line_filtered_error;
}

static void line_pwm_control(void)
{
    uint8_t bits = xunji_line_bits();
    int count = sensor_count_bits(bits);
    int raw_error;
    int error;
    int base;
    int corr;
    int diff;

    if(s_line_sharp_state != 0)
    {
        if(s_line_sharp_brake_ticks > 0)
        {
            s_line_sharp_brake_ticks--;
            line_brake_control();
            return;
        }

        if(count == 0)
        {
            s_line_sharp_exit_ticks = 0;
            s_line_derivative_valid = 0;
            line_sharp_turn_control(LINE_SHARP_LOST_PWM);
            return;
        }

        raw_error = line_error_bits(bits);
        s_line_last_error = raw_error;
        s_line_has_last = 1;

        if(line_center_seen_bits(bits) && abs_int(raw_error) <= LINE_TURN_SLOW_ERROR)
        {
            s_line_sharp_exit_ticks++;
            if(s_line_sharp_exit_ticks >= LINE_SHARP_EXIT_TICKS)
            {
                s_line_sharp_state = 0;
                s_line_sharp_exit_ticks = 0;
                s_line_last_valid_error = raw_error;
                s_line_filtered_error = raw_error;
                s_line_filter_valid = 1;
            }
            else
            {
                line_sharp_turn_control(LINE_SHARP_LOST_PWM);
                return;
            }
        }
        else
        {
            s_line_sharp_exit_ticks = 0;
            line_sharp_turn_control(LINE_SHARP_PWM);
            return;
        }
    }

    if(count == 0)
    {
        s_line_derivative_valid = 0;
        s_line_filter_valid = 0;
        line_search_control();
        return;
    }

    raw_error = line_error_bits(bits);
    s_line_last_error = raw_error;
    s_line_has_last = 1;

    if(raw_error <= -LINE_SHARP_ERROR)
    {
        s_line_sharp_state = -1;
        s_line_sharp_brake_ticks = LINE_SHARP_BRAKE_TICKS;
        s_line_sharp_exit_ticks = 0;
        s_line_last_valid_error = raw_error;
        s_line_filter_valid = 0;
        line_brake_control();
        return;
    }
    else if(raw_error >= LINE_SHARP_ERROR)
    {
        s_line_sharp_state = 1;
        s_line_sharp_brake_ticks = LINE_SHARP_BRAKE_TICKS;
        s_line_sharp_exit_ticks = 0;
        s_line_last_valid_error = raw_error;
        s_line_filter_valid = 0;
        line_brake_control();
        return;
    }

    error = line_filter_error(raw_error);

    if(s_line_derivative_valid == 0)
    {
        diff = 0;
        s_line_derivative_valid = 1;
    }
    else
    {
        diff = clamp_int(error - s_line_last_valid_error,
                         -LINE_DIFF_LIMIT,
                         LINE_DIFF_LIMIT);
    }
    s_line_last_valid_error = error;
    base = (error > LINE_TURN_SLOW_ERROR || error < -LINE_TURN_SLOW_ERROR) ? LINE_TURN_BASE_PWM : LINE_BASE_PWM;
    corr = clamp_int(error * LINE_KP + diff * LINE_KD, -LINE_PWM_LIMIT, LINE_PWM_LIMIT);
    line_set_pwm(base + corr, base - corr);
}

static void update_cross_count(void)
{
    uint8_t cross_detected =
        (sensor_count_bits(xunji_line_bits()) >= LINE_CROSS_COUNT) ? 1 : 0;

    if(cross_detected)
    {
        s_line_cross_low_ticks = 0;
        if(s_line_cross_latched == 0)
        {
            if(s_line_cross_high_ticks < LINE_CROSS_CONFIRM_TICKS)
            {
                s_line_cross_high_ticks++;
            }

            if(s_line_cross_high_ticks >= LINE_CROSS_CONFIRM_TICKS)
            {
                s_line_cross_latched = 1;
                change_flag1++;
            }
        }
    }
    else
    {
        s_line_cross_high_ticks = 0;
        if(s_line_cross_latched)
        {
            if(s_line_cross_low_ticks < LINE_CROSS_RELEASE_TICKS)
            {
                s_line_cross_low_ticks++;
            }

            if(s_line_cross_low_ticks >= LINE_CROSS_RELEASE_TICKS)
            {
                s_line_cross_latched = 0;
            }
        }
    }

    now_statue = s_line_cross_latched;
    last_statue = now_statue;
}

void track1(void)
{
    line_pwm_control();
}

void track2(void)
{
    update_cross_count();

    if(change_flag1 >= 4)
    {
        Set_left_pwm(0);
        Set_right_pwm(0);
    }
    else if(change_flag1 >= 2)
    {
        turn_pid(line_drive_speed(TURN_BASE_SPEED), 177);
    }
    else
    {
        line_pwm_control();
    }
}

void track3(void)
{
    update_cross_count();

    if(change_flag1 >= 4)
    {
        Set_left_pwm(0);
        Set_right_pwm(0);
    }
    else if(change_flag1 >= 2)
    {
        turn_pid(line_drive_speed(TURN_BASE_SPEED), -137);
    }
    else
    {
        line_pwm_control();
    }
}

void track4(void)
{
    update_cross_count();

    if(change_flag1 >= 16)
    {
        Set_left_pwm(0);
        Set_right_pwm(0);
    }
    else if(change_flag1 >= 14)
    {
        turn_pid(line_drive_speed(TURN_BASE_SPEED), -142);
    }
    else if(change_flag1 >= 12)
    {
        turn_pid(line_drive_speed(TURN_BASE_SPEED), -50);
    }
    else if(change_flag1 >= 10)
    {
        turn_pid(line_drive_speed(TURN_BASE_SPEED), -142);
    }
    else if(change_flag1 >= 8)
    {
        turn_pid(line_drive_speed(TURN_BASE_SPEED), -47);
    }
    else if(change_flag1 >= 6)
    {
        turn_pid(line_drive_speed(TURN_BASE_SPEED), -141);
    }
    else if(change_flag1 >= 4)
    {
        turn_pid(line_drive_speed(TURN_BASE_SPEED), -46);
    }
    else if(change_flag1 >= 2)
    {
        turn_pid(line_drive_speed(TURN_BASE_SPEED), -138);
    }
    else
    {
        line_pwm_control();
    }
}
