
#include "MyProject.h"

/******************************************************************************/
EncoderDiagnostics_t encoder_diag[3];
uint32_t encoder_diag_epoch;
void reset_encoder_diagnostics(void) {
    int i;
    /* Keep safety freshness timestamps valid even when FREQ is reset while running. */
    for(i=0;i<3;i++) {
        encoder_diag[i].samples=encoder_diag[i].errors=0;
        encoder_diag[i].max_gap_us=encoder_diag[i].max_read_us=0;
    }
    encoder_diag_epoch=micros();
}
float shaft_angle[3] = {0.0f, 0.0f, 0.0f};      //!< current motor continuous angle (rad)
float electrical_angle[3] = {0.0f, 0.0f, 0.0f};
float shaft_velocity[3] = {0.0f, 0.0f, 0.0f};   //!< measured filtered velocity (rad/s)
float current_sp = 0.0f;
float shaft_velocity_sp[3] = {0.0f, 0.0f, 0.0f};
float shaft_angle_sp[3] = {0.0f, 0.0f, 0.0f};
DQVoltage_s voltage[3];
DQCurrent_s current;

TorqueControlType torque_controller = Type_voltage;
MotionControlType controller = Type_velocity; // 默认启动速度闭环控制模式

float sensor_offset[3] = {0.0f, 0.0f, 0.0f};
float zero_electric_angle[3] = {0.0f, 0.0f, 0.0f};
int   sensor_direction[3] = {1, 1, 1}; // 默认 CW
uint8_t motor_aligned[3] = {0, 0, 0};
uint8_t sensor_valid[3] = {0, 0, 0};
uint8_t motor_sensor_map[3] = {0, 1, 2};
uint8_t motor_mapping_valid[3] = {0, 0, 0};

/* 编码器跟踪状态内部结构 */
float angle_prev[3] = {0.0f, 0.0f, 0.0f};           // 当前单圈机械角度 [0, 2PI)
static unsigned long angle_prev_ts[3] = {0, 0, 0};   // 上一次角度采样时间戳
static float angle_vel_prev[3] = {0.0f, 0.0f, 0.0f}; // 上一次计算速度时的单圈角度 (严格时间对齐)
static unsigned long vel_prev_ts[3] = {0, 0, 0};     // 上一次计算速度时的时间戳
static float last_raw_velocity[3] = {0.0f, 0.0f, 0.0f};

PIDController_t pid_velocity[3];
LowPassFilter_t lpf_velocity[3];

/* 闭环虚拟同步旋转磁场电角度与微秒时间戳 */
float target_angle_el[3] = {0.0f, 0.0f, 0.0f};
unsigned long move_timestamp_prev[3] = {0, 0, 0};

/******************************************************************************/
/* 复位指定电机的闭环同步电角度与 PID/滤波状态 (motor: 0~2为指定轴, -1为全部) */
void reset_closed_loop_state(int motor)
{
	int start = (motor >= 0 && motor < 3) ? motor : 0;
	int end   = (motor >= 0 && motor < 3) ? motor : 2;
	int k;
	for(k = start; k <= end; k++)
	{
		float rotor_el = (float)sensor_direction[k] * shaft_angle[k] * (float)motor_pole_pairs[k] - zero_electric_angle[k];
		target_angle_el[k] = rotor_el;
		move_timestamp_prev[k] = micros();
		pid_velocity[k].error_prev = 0.0f;
		pid_velocity[k].output_prev = 0.0f;
		pid_velocity[k].integral_prev = 0.0f;
		pid_velocity[k].timestamp_prev = 0;
		lpf_velocity[k].y_prev = 0.0f;
		lpf_velocity[k].timestamp_prev = 0;
		shaft_velocity[k] = 0.0f;
		shaft_velocity_sp[k] = 0.0f;
	}
}

/******************************************************************************/
/* 初始化速度环 PID 和低通滤波器参数 */
void SimpleFOC_PID_Init(void)
{
	int i;
	for(i = 0; i < 3; i++)
	{
		// 速度环 PID 参数：配合反电势前馈与全速域混合架构
		pid_velocity[i].P = i==1?0.20f:0.30f;           // 适中比例增益
		pid_velocity[i].I = i==1?1.50f:0.80f;           // 稳健积分增益
		pid_velocity[i].D = 0.000f;          // 速度环置零 D 项，彻底消除高频毛刺与抖动
		pid_velocity[i].output_ramp = 0.0f;  // 不人为限制输出斜率，消除相角滞后
		pid_velocity[i].limit = 6.0f;        // 默认与 voltage_limit (6.0V) 对齐，释放充沛电磁转矩
		pid_velocity[i].error_prev = 0.0f;
		pid_velocity[i].output_prev = 0.0f;
		pid_velocity[i].integral_prev = 0.0f;
		pid_velocity[i].timestamp_prev = 0;

		// M1/M3按12V实机阶跃整定为4ms；M2保留12ms，待硬件检修后单独整定
		lpf_velocity[i].Tf = i==1?0.012f:0.004f;
		lpf_velocity[i].y_prev = 0.0f;
		lpf_velocity[i].timestamp_prev = 0;
	}
}

/* Discard angle and velocity history after a drive-to-encoder remap. */
void reset_sensor_tracking(int motor)
{
	if(motor < 0 || motor > 2) return;
	angle_prev_ts[motor] = 0;
	angle_vel_prev[motor] = 0.0f;
	vel_prev_ts[motor] = 0;
	last_raw_velocity[motor] = 0.0f;
	angle_prev[motor] = 0.0f;
	shaft_angle[motor] = 0.0f;
	shaft_velocity[motor] = 0.0f;
	lpf_velocity[motor].y_prev = 0.0f;
	lpf_velocity[motor].timestamp_prev = 0;
	sensor_valid[motor] = 0;
	updateSensor(motor);
}

/******************************************************************************/
/* 从 MT6701 读取并更新多圈角度与滤波后速度 */
void updateSensor(int motor)
{
	int16_t raw = 0;
	float deg = 0.0f;
	uint8_t res = 1;
	float rad = 0.0f;
	float d_angle = 0.0f;
	uint32_t read_start=micros(),now_us=read_start;
	uint32_t d_us = 0;
	float Ts = 0.0f;
	float raw_vel = 0.0f;
    float counts_per_rev;

	if(motor < 0 || motor > 2) return;
	uint8_t sens_idx = motor_sensor_map[motor];

	/* 1. 读取映射对应的编码器通道
	 * 统一经由 ENC_GetAngle 抽象层: 编译期宏 ENC_USE_HW_QUAD 决定
	 * 三路 I2C 绝对式读取 (原方法) 或 TIM2/TIM5/TIM8 硬件正交解码 (新方法)
	 */
	res = ENC_GetAngle(sens_idx, &raw, &deg);
    now_us=micros();
    if(now_us-read_start>encoder_diag[motor].max_read_us) encoder_diag[motor].max_read_us=now_us-read_start;

    counts_per_rev=(float)ENC_CountsPerRev(sens_idx);
	if(res != 0 || counts_per_rev<=0 || raw<0 || (float)raw>=counts_per_rev)
	{
		sensor_valid[motor] = 0;
        encoder_diag[motor].errors++;
		return;
	}
	sensor_valid[motor] = 1;
    if(encoder_diag[motor].last_us && now_us-encoder_diag[motor].last_us>encoder_diag[motor].max_gap_us)
        encoder_diag[motor].max_gap_us=now_us-encoder_diag[motor].last_us;
    encoder_diag[motor].last_us=now_us;encoder_diag[motor].samples++;

	/* 转换到单圈绝对弧度 [0, 2PI)
	 * I2C 模式: 14-bit 绝对值 (16384 counts/圈)
	 * 正交解码模式: 按该传感器的独立CPR缩放，计数器在一机械圈回绕
	 */
	rad=(float)raw*_2PI/counts_per_rev;
    now_us=micros();

	// 2. 初始帧采样
	if(angle_prev_ts[motor] == 0)
	{
		angle_prev[motor] = rad;
		angle_vel_prev[motor] = rad;
		angle_prev_ts[motor] = now_us;
		vel_prev_ts[motor] = now_us;
		shaft_angle[motor] = rad;
		shaft_velocity[motor] = 0.0f;
		return;
	}

	// 3. 经典圆周跨界最短角位移计算 (0 <-> 2PI 跨越保护)
	d_angle = rad - angle_prev[motor];
	if (d_angle > _PI) d_angle -= _2PI;
	else if (d_angle < -_PI) d_angle += _2PI;

	angle_prev[motor] = rad;
	angle_prev_ts[motor] = now_us;

	// 4. 连续多圈绝对机械角位移平滑累加
	shaft_angle[motor] += d_angle;

	// 5. 对齐官方 SimpleFOC Sensor::getVelocity 高精度自适应测速引擎
	// 避免固定 5ms 强制截断引起的离散零阶跃跳变与量化毛刺
	d_us = now_us - (uint32_t)vel_prev_ts[motor];

	if(d_us >= 1000) // 采样窗口至少 1ms 以上
	{
		float d_vel = rad - angle_vel_prev[motor];
		if (d_vel > _PI) d_vel -= _2PI;
		else if (d_vel < -_PI) d_vel += _2PI;

		// 随当前轴分辨率设置半LSB门槛，兼容I2C与不同PPR的正交输入
		if(fabsf(d_vel) >= 0.5f*_2PI/counts_per_rev)
		{
			Ts = (float)d_us * 1e-6f;
			raw_vel = (float)sensor_direction[motor] * (d_vel / Ts);

			angle_vel_prev[motor] = rad;
			vel_prev_ts[motor] = now_us;

			// 极值限幅保护
			if(raw_vel > 600.0f) raw_vel = 600.0f;
			else if(raw_vel < -600.0f) raw_vel = -600.0f;
			last_raw_velocity[motor] = raw_vel;

			// 一阶低通滤波 (Tf = 12ms)
			shaft_velocity[motor] = LPF_operator(&lpf_velocity[motor], raw_vel);
		}
		else
		{
			uint32_t stop_timeout_us = 25000U;

			if(fabsf(last_raw_velocity[motor]) > 0.0f)
			{
				float pulse_period_us = _2PI * 1e6f /
					(counts_per_rev * fabsf(last_raw_velocity[motor]));
				float timeout = _constrain(3.0f * pulse_period_us, 25000.0f, 1000000.0f);
				stop_timeout_us = (uint32_t)timeout;
			}

			if(d_us >= stop_timeout_us)
			{
				angle_vel_prev[motor] = rad;
				vel_prev_ts[motor] = now_us;
				last_raw_velocity[motor] = 0.0f;
				shaft_velocity[motor] = LPF_operator(&lpf_velocity[motor], 0.0f);
			}
		}
	}
}

/******************************************************************************/
float shaftAngle(int motor)
{
	if(motor >= 0 && motor < 3) return shaft_angle[motor];
	return 0.0f;
}

/******************************************************************************/
float shaftVelocity(int motor)
{
	if(motor >= 0 && motor < 3) return shaft_velocity[motor];
	return 0.0f;
}

/******************************************************************************/
/* 计算电机当前转子电角度 (考虑旋转方向和对齐零电角)
 * 直接使用当前单圈高精度机械弧度 angle_prev[motor]，无多圈累积误差与跨圈跳变
 */
float electricalAngle(int motor)
{
	if(motor < 0 || motor > 2) return 0.0f;
	return _normalizeAngle((float)(sensor_direction[motor]) * (angle_prev[motor] * (float)motor_pole_pairs[motor]) - zero_electric_angle[motor]);
}
/******************************************************************************/


