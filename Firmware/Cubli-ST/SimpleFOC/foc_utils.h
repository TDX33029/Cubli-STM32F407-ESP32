#ifndef FOCUTILS_LIB_H
#define FOCUTILS_LIB_H

#include <math.h>

/******************************************************************************/
// sign function
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _round(x) ((x)>=0.0f?(long)((x)+0.5f):(long)((x)-0.5f))
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ( (a) != (NOT_SET) )

// utility defines
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#ifndef _PI
#define _PI 3.14159265359f
#endif
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
/******************************************************************************/
// dq current structure
typedef struct
{
	float d;
	float q;
} DQCurrent_s;
// phase current structure
typedef struct
{
	float a;
	float b;
	float c;
} PhaseCurrent_s;
// dq voltage structs
typedef struct
{
	float d;
	float q;
} DQVoltage_s;

// PID 控制器结构体
typedef struct
{
	float P;             // 比例增益
	float I;             // 积分增益
	float D;             // 微分增益
	float output_ramp;   // 输出变化斜率限制 (V/s, 0表示不限制)
	float limit;         // 输出最大限制 (V)
	float error_prev;    // 上一次误差
	float output_prev;   // 上一次输出
	float integral_prev; // 上一次积分项累加值
	unsigned long timestamp_prev; // 上一次计算的时间戳 (SysTick ticks)
} PIDController_t;

// 一阶低通滤波器结构体
typedef struct
{
	float Tf;            // 滤波时间常数 (s)
	float y_prev;        // 上一次滤波输出
	unsigned long timestamp_prev; // 上一次滤波时间戳 (SysTick ticks)
} LowPassFilter_t;

/******************************************************************************/
float _sin(float a);
float _cos(float a);
float _normalizeAngle(float angle);
float _electricalAngle(float shaft_angle, int pole_pairs);
float _sqrtApprox(float number);
float PID_operator(PIDController_t* pid, float error);
float PID_operator_offset(PIDController_t* pid, float error, float output_offset);
float LPF_operator(LowPassFilter_t* lpf, float x);
/******************************************************************************/


#endif
