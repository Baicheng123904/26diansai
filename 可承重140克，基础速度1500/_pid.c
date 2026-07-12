#include "headfile.h"

#define motor_max_duty 9999

pid_t motorA;
pid_t motorB;
pid_t angle;
int left_encoder,right_encoder;
extern unsigned int Temp[2];
int A_turn,B_turn;


void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
	pid->pid_mode = mode;
	pid->p = p;
	pid->i = i;
	pid->d = d;
	pid_runtime_reset(pid);
}

void pid_runtime_reset(pid_t *pid)
{
    pid->target = 0;
    pid->now = 0;
    pid->error[0] = 0;
    pid->error[1] = 0;
    pid->error[2] = 0;
    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out = 0;
}

void pid_reset_all(void)
{
    pid_runtime_reset(&motorA);
    pid_runtime_reset(&motorB);
    pid_runtime_reset(&angle);
}


void motor_target_set(int spe1, int spe2)
{
	if(spe1 >= 0)
	{
		motorA_dir = 1;
		motorA.target = spe1;
	}
	else
	{
		motorA_dir = 0;
		motorA.target = -spe1;
	}
	
	if(spe2 >= 0)
	{
		motorB_dir = 1;
		motorB.target = spe2;
	}
	else
	{
		motorB_dir = 0;
		motorB.target = -spe2;
	}
}

//void pid_control()
//{
//	// 1.设定目标速度
//	motor_target_set(20, 20);
//	// 2.获取当前速度
//	if(motorA_dir){motorA.now = Encoder_count1;}else{motorA.now = -Encoder_count1;}
//	if(motorB_dir){motorB.now = Encoder_count2;}else{motorB.now = -Encoder_count2;}
//	Encoder_count1 = 0;
//	Encoder_count2 = 0;
//	// 3.输入PID控制器进行计算
//	pid_cal(&motorA);
//	pid_cal(&motorB);
//	// 4.PID的输出值 输入给电机
//	motorA_duty(motorA.out);
//	motorB_duty(motorB.out);
//}


void pid_cal(pid_t *pid)
{
	// 计算当前偏差
	pid->error[0] = pid->target - pid->now;

	// 计算输出
	if(pid->pid_mode == DELTA_PID)         // 增量式
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if(pid->pid_mode == POSITION_PID)  // 位置式
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	// 记录前两次偏差
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	
}

void pid_out_limit(pid_t *pid)
{
	 	// 输出限幅
	if(pid->out>=MAX_DUTY)	
		pid->out=MAX_DUTY;
	if(pid->out<=-MAX_DUTY)	
		pid->out=-MAX_DUTY;
}

static int yaw_error_wrap(int target, int current)
{
    int error = target - current;

    while(error > 180)
    {
        error -= 360;
    }

    while(error < -180)
    {
        error += 360;
    }

    return error;
}


void turn_pid(int base,int target)
{
		/* Wrap the control error only; preserve the IMU measurement. */
	 	angle.now=0;
		angle.target=yaw_error_wrap(target, yaw_angle_int);
		pid_cal(&angle);
		motorA.now=left_encoder;
		motorA.target=base-angle.out;
		motorB.now=right_encoder;
		motorB.target=base+angle.out;
		pid_cal(&motorA);
		pid_cal(&motorB);
		pid_out_limit(&motorA);
		pid_out_limit(&motorB);
		
		Set_left_pwm((int)motorA.out);
    Set_right_pwm((int)motorB.out);
	
}

void speed_pid(int target)
{
		  motorA.now=left_encoder;
		  motorA.target=target;
		  pid_cal(&motorA);
			motorB.now=right_encoder;
		  motorB.target=target;
		  pid_cal(&motorB);
			pid_out_limit(&motorA);
			pid_out_limit(&motorB);
		Set_left_pwm((int)motorA.out);
	  Set_right_pwm((int)motorB.out);
}


void track_pid(int temp)
{
	  motorA.error[0]=temp;
	  motorB.error[0]=temp;
	  pid_cal(&motorA);
	  pid_cal(&motorB);
}



