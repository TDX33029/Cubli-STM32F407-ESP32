
#include "MyProject.h"

/***************************************************************************/
// int array instead of float array
// 4x200 points per 360 deg
// 2x storage save (int 2Byte float 4 Byte )
// sin*10000
const int sine_array[200] = {0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000};

/***************************************************************************/
// 高精度连续正弦函数 (基于 STM32F407 硬件 FPU 单元加速，精度极高，无查表离散阶跃)
float _sin(float a){
  return sinf(a);
}
/***************************************************************************/
// 高精度连续余弦函数 (基于 STM32F407 硬件 FPU 单元加速)
float _cos(float a){
  return cosf(a);
}
/***************************************************************************/
// normalizing radian angle to [0,2PI]
float _normalizeAngle(float angle){
  float a = fmodf(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}
/***************************************************************************/
// Electrical angle calculation
float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}
/***************************************************************************/
// square root approximation function using
// https://reprap.org/forum/read.php?147,219210
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
float _sqrtApprox(float number) {//low in fat
  long i;
  float y;
  // float x;
  // const float f = 1.5F; // better precision

  // x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  // y = y * ( f - ( x * y * y ) ); // better precision
  return number * y;
}
/***************************************************************************/
/* PID 速度控制器运算 (基于 TIM5 32位全硬件微秒时钟) */
float PID_operator(PIDController_t* pid, float error)
{
	return PID_operator_offset(pid, error, 0.0f);
}

float PID_operator_offset(PIDController_t* pid, float error, float output_offset)
{
	uint32_t now_us = micros();
	float Ts;
	float proportional, integral, derivative, output;
	float output_rate;

	if(pid->timestamp_prev == 0)
	{
		Ts = 1e-3f;
	}
	else
	{
		uint32_t d_us = now_us - (uint32_t)pid->timestamp_prev;
		Ts = (float)d_us * 1e-6f;
	}
	if(Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;
	pid->timestamp_prev = now_us;

	// 比例项
	proportional = pid->P * error;

	output_offset = _constrain(output_offset, -pid->limit, pid->limit);

	// 积分项 (条件抗饱和同时考虑反电势/摩擦前馈占用的输出裕量)
	float new_integral = pid->integral_prev + pid->I * error * Ts;
	new_integral = _constrain(new_integral, -pid->limit, pid->limit);
	if(!((proportional + new_integral + output_offset >= pid->limit && error > 0.0f) ||
	     (proportional + new_integral + output_offset <= -pid->limit && error < 0.0f)))
	{
		pid->integral_prev = new_integral;
	}
	integral = pid->integral_prev;

	// 微分项 (带安全滤波与微分限幅，D=0时直接为0，绝不引入高频尖峰)
	if(pid->D > 0.0f && Ts > 0.0001f)
	{
		derivative = pid->D * (error - pid->error_prev) / Ts;
		derivative = _constrain(derivative, -1.5f, 1.5f); // 限制微分贡献不超过 0.5V
	}
	else
	{
		derivative = 0.0f;
	}
	pid->error_prev = error;

	output = proportional + integral + derivative;
	output = _constrain(output, -pid->limit - output_offset, pid->limit - output_offset);

	// 输出变化斜率限制 (Ramp)
	if(pid->output_ramp > 0.0f && Ts > 0.0005f)
	{
		output_rate = (output - pid->output_prev) / Ts;
		if(output_rate > pid->output_ramp)
			output = pid->output_prev + pid->output_ramp * Ts;
		else if(output_rate < -pid->output_ramp)
			output = pid->output_prev - pid->output_ramp * Ts;
	}
	pid->output_prev = output;
	return output;
}

/***************************************************************************/
/* 一阶低通滤波算子 (基于 TIM5 32位全硬件微秒时钟) */
float LPF_operator(LowPassFilter_t* lpf, float x)
{
	uint32_t now_us = micros();
	float Ts, alpha, y;

	if(lpf->timestamp_prev == 0)
	{
		Ts = 5e-3f;
	}
	else
	{
		uint32_t d_us = now_us - (uint32_t)lpf->timestamp_prev;
		Ts = (float)d_us * 1e-6f;
	}
	if(Ts <= 0.0f || Ts > 0.5f) Ts = 5e-3f;
	lpf->timestamp_prev = now_us;

	if(lpf->Tf <= 0.0f) return x;
	alpha = lpf->Tf / (lpf->Tf + Ts);
	y = alpha * lpf->y_prev + (1.0f - alpha) * x;
	lpf->y_prev = y;
	return y;
}
