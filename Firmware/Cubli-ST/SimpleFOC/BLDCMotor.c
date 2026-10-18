
#include "MyProject.h"
#include "lqr_balance.h"

extern void set_motor_enable(uint8_t motor, uint8_t en);
extern uint8_t Control_Aborted(void);
extern void Control_RequestStop(uint8_t reason);

/************************************************
main中调用的接口函数都在当前文件中
=================================================
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/119220638
创建日期：20210801
作    者：loop222 @郑州
  F4移植: 适配168MHz, TIM1 PWM (PE9/PE11/PE13)
************************************************/
/******************************************************************************/
float motor_test_travel_deg[3]={0,0,0};
float voltage_power_supply;
float voltage_limit;
int  pole_pairs;
uint8_t motor_pole_pairs[3]={MOTOR_PP_M1,MOTOR_PP_M2,MOTOR_PP_M3};
unsigned long open_loop_timestamp[3];
float velocity_limit;
float velocity_ramp_accel = 400.0f;
float voltage_sensor_align = 3.5f; // 单轴短时对齐电压；提高负载下的扫描跟随余量
extern float open_loop_angle[3];
/******************************************************************************/
float velocityOpenloop(float target_velocity, int motor);
float angleOpenloop(float target_angle, int motor);

static uint8_t detect_motor_encoder(int motor)
{
	int i, matched = -1;
	float saved_voltage_limit = voltage_limit;
	float previous_deg[3] = {0.0f, 0.0f, 0.0f};
	float travel_deg[3] = {0.0f, 0.0f, 0.0f};
	float signed_travel[3] = {0.0f, 0.0f, 0.0f};
	uint16_t success[3] = {0, 0, 0};
	uint8_t previous_valid[3] = {0, 0, 0};
	uint16_t samples = 0;
	uint32_t start_tick, last_sample_tick;
	uint8_t aborted = 0;

	if(motor < 0 || motor > 2) return 0;
	motor_mapping_valid[motor] = 0;
	set_motor_enable(0, 0);
	voltage_limit = fminf(3.0f,saved_voltage_limit);
	open_loop_angle[motor] = 0.0f;
	open_loop_timestamp[motor] = micros();
	set_motor_enable(motor + 1, 1);
	delay_ms(20);

	start_tick = HAL_GetTick();
	last_sample_tick = start_tick;
	while((HAL_GetTick() - start_tick) < 2000U)
	{
		if(Control_Aborted()) {aborted = 1; break;}
		velocityOpenloop(2.0f, motor);

		if((HAL_GetTick() - last_sample_tick) >= 20U)
		{
			last_sample_tick = HAL_GetTick();
			samples++;
			for(i = 0; i < 3; i++)
			{
				int16_t raw = 0;
				float angle_deg = 0.0f;
				if(ENC_GetAngle((uint8_t)i, &raw, &angle_deg) == 0)
				{
					success[i]++;
					if(previous_valid[i])
					{
						float diff = angle_deg - previous_deg[i];
						if(diff > 180.0f) diff -= 360.0f;
						else if(diff < -180.0f) diff += 360.0f;
						travel_deg[i] += fabsf(diff);
						signed_travel[i] += diff;
					}
					previous_deg[i] = angle_deg;
					previous_valid[i] = 1;
				}
				else previous_valid[i] = 0;
			}
		}
		delay_us(500);
	}

	setPhaseVoltage(0.0f, 0.0f, 0.0f, motor);
	set_motor_enable(motor + 1, 0);
	voltage_limit = saved_voltage_limit;

	if(aborted)
	{
		printf("ERR MAP M%d: SAFETY_ABORT\r\n", motor + 1);
		return 0;
	}

	for(i = 0; i < 3; i++)
	{
		uint32_t read_pct = samples ? ((uint32_t)success[i] * 100U) / samples : 0U;
		if(travel_deg[i] >= 20.0f && read_pct >= 80U &&
		   (matched < 0 || travel_deg[i] > travel_deg[matched])) matched = i;
	}

	if(matched < 0)
	{
		motor_test_travel_deg[motor] = travel_deg[0];
		for(i = 1; i < 3; i++)
			if(travel_deg[i] > motor_test_travel_deg[motor]) motor_test_travel_deg[motor] = travel_deg[i];
		printf("ERR MAP M%d: no encoder followed; ENC1=%.1f/%.0f%% ENC2=%.1f/%.0f%% ENC3=%.1f/%.0f%%\r\n",
			motor + 1, travel_deg[0], samples ? 100.0f * success[0] / samples : 0.0f,
			travel_deg[1], samples ? 100.0f * success[1] / samples : 0.0f,
			travel_deg[2], samples ? 100.0f * success[2] / samples : 0.0f);
		return 0;
	}

	for(i = 0; i < 3; i++)
	{
		if(i != motor && motor_mapping_valid[i] && motor_sensor_map[i] == (uint8_t)matched)
		{
			printf("ERR MAP M%d: ENC%d already assigned to M%d\r\n", motor + 1, matched + 1, i + 1);
			return 0;
		}
	}

	motor_sensor_map[motor] = (uint8_t)matched;
	motor_mapping_valid[motor] = 1;
	motor_test_travel_deg[motor] = travel_deg[matched];
	if(signed_travel[matched] != 0.0f)
		sensor_direction[motor] = signed_travel[matched] > 0.0f ? 1 : -1;
	reset_sensor_tracking(motor);
	printf("OK MAP M%d -> ENC%d, direction=%d, travel=%.1fdeg, reads=%u/%u\r\n",
		motor + 1, matched + 1, sensor_direction[motor], travel_deg[matched], success[matched], samples);
	return 1;
}

/******************************************************************************/
/* 对单个电机进行电角度零位对齐与旋转方向辨识 (严格遵循官方 SimpleFOC 闭环校准算法) */
static uint8_t align_wait_still(int motor,float *angle)
{
    int window,sample,stable=0;
    float previous=0;
    /* A fixed delay can end at a flywheel oscillation peak. Require two
       consecutive 80ms windows with the existing 0.02rad spread bound. */
    for(window=0;window<20;window++) {
        float mean=0,lo=0,hi=0;
        for(sample=0;sample<40;sample++) {
            float current_angle;
            updateSensor(motor);
            if(!sensor_valid[motor] || Control_Aborted()) return 0;
            current_angle=shaftAngle(motor);
            if(!sample) lo=hi=current_angle;
            lo=fminf(lo,current_angle);hi=fmaxf(hi,current_angle);
            mean+=current_angle;delay_ms(2);
        }
        mean/=40.0f;
        if(hi-lo<=0.02f) {
            stable=(stable && fabsf(mean-previous)<=0.01f)?stable+1:1;
            if(stable>=2) {*angle=mean;return 1;}
        } else stable=0;
        previous=mean;
    }
    return 0;
}
uint8_t Motor_alignSensor(int motor)
{
    int i; float start,finish,forward,reverse,pp_est,align_v,settled;
    if(motor<0 || motor>2) return 1;
    LQR_InvalidateMotorCalibration((uint8_t)motor);
    motor_aligned[motor]=0;motor_mapping_valid[motor]=0;motor_test_travel_deg[motor]=0;
    if(!detect_motor_encoder(motor)) return 1;
    align_v=fminf(voltage_sensor_align,voltage_limit);
    set_motor_enable(motor+1,1);
    setPhaseVoltage(align_v,0,_3PI_2,motor);delay_ms(500);updateSensor(motor);
    if(!sensor_valid[motor] || Control_Aborted()) goto failed;
    if(!align_wait_still(motor,&start)) {printf("ERR ALIGN_UNSTABLE M%d stage=start\r\n",motor+1);goto failed;}
    /* Independent, slow forward/backward electrical sweep. Open-loop mapping
       travel is NOT sufficient to establish commutation direction or pole count. */
    for(i=1;i<=120;i++) {
        setPhaseVoltage(align_v,0,_3PI_2+_2PI*i/120.0f,motor);
        delay_ms(10);updateSensor(motor);
        if(!sensor_valid[motor] || Control_Aborted()) goto failed;
        if((i%20)==0) printf("$SCAN,M:%d,Stage:F,Step:%d,Angle:%.4f#\r\n",motor+1,i,shaftAngle(motor));
    }
    delay_ms(200);
    if(!align_wait_still(motor,&finish)) {printf("ERR ALIGN_UNSTABLE M%d stage=forward\r\n",motor+1);goto failed;}
    forward=finish-start;
    for(i=119;i>=0;i--) {
        setPhaseVoltage(align_v,0,_3PI_2+_2PI*i/120.0f,motor);
        delay_ms(10);updateSensor(motor);
        if(!sensor_valid[motor] || Control_Aborted()) goto failed;
        if((i%20)==0) printf("$SCAN,M:%d,Stage:R,Step:%d,Angle:%.4f#\r\n",motor+1,i,shaftAngle(motor));
    }
    delay_ms(200);
    if(!align_wait_still(motor,&settled)) {printf("ERR ALIGN_UNSTABLE M%d stage=reverse\r\n",motor+1);goto failed;}
    reverse=settled-finish;
    if(!sensor_valid[motor] || Control_Aborted()) goto failed;
    if(fabsf(forward)<0.05f || fabsf(reverse)<0.05f || forward*reverse>=0 ||
       fabsf(fabsf(forward)-fabsf(reverse))>0.2f*fmaxf(fabsf(forward),fabsf(reverse))) {
        printf("ERR ALIGN_FOLLOW M%d fwd=%.3f rev=%.3f rad\r\n",motor+1,forward,reverse);goto failed;
    }
    pp_est=_2PI/(0.5f*(fabsf(forward)+fabsf(reverse)));
    printf("[ALIGN] M%d PP_est=%.2f configured=%d fwd=%.3f rev=%.3f\r\n",motor+1,pp_est,motor_pole_pairs[motor],forward,reverse);
    if((int)(pp_est+0.5f)!=motor_pole_pairs[motor]) {
        printf("ERR ALIGN_PP M%d expected=%d measured=%.2f (check PP, phases and magnet)\r\n",motor+1,motor_pole_pairs[motor],pp_est);goto failed;
    }
    sensor_direction[motor]=forward>0?1:-1;
    zero_electric_angle[motor]=_normalizeAngle(sensor_direction[motor]*settled*motor_pole_pairs[motor]);
    set_motor_enable(motor+1,0);reset_closed_loop_state(motor);reset_motion_state(motor);
    motor_aligned[motor]=1;
    printf("Align M%d OK (ENC%d): Dir=%d, PP=%d, Zero_Elec=%.4f rad\r\n",motor+1,motor_sensor_map[motor]+1,sensor_direction[motor],motor_pole_pairs[motor],zero_electric_angle[motor]);
    return 0;
failed:
    set_motor_enable(motor+1,0);motor_aligned[motor]=0;
    return 1;
}

/******************************************************************************/
/* 对所有三轴电机执行一键顺序对齐校准 */
void Motor_alignAll(void)
{
	printf("\r\n=== Starting 3-Axis MT6701 Sensor Align ===\r\n");
	Motor_alignSensor(0);
	Motor_alignSensor(1);
	Motor_alignSensor(2);

	// 复位三轴速度环与闭环同步电角度状态，杜绝切入闭环瞬间产生扰动
	// 校准完毕后，强制切断全部三轴电机驱动硬件使能与相电压输出，处于安全待机状态
	set_motor_enable(0, 0);
	setPhaseVoltage(0.0f, 0.0f, 0.0f, 0);
	setPhaseVoltage(0.0f, 0.0f, 0.0f, 1);
	setPhaseVoltage(0.0f, 0.0f, 0.0f, 2);

	reset_closed_loop_state(-1);

	printf("=== 3-Axis Sensor Align Finished ===\r\n\r\n");
}

/******************************************************************************/
/* FOC 内环：读取传感器并更新电机力矩/相电压换向 */
void loopFOC(int motor)
{
	if(motor < 0 || motor > 2) return;
	// 更新编码器多圈绝对角度与角速度
	updateSensor(motor);
	// 计算转子电角度
	electrical_angle[motor] = electricalAngle(motor);
}

/******************************************************************************/
float open_loop_angle[3] = {0.0f, 0.0f, 0.0f};
static uint32_t velocity_ramp_timestamp[3] = {0, 0, 0};
static uint32_t locked_rotor_since[3] = {0, 0, 0};
void reset_motion_state(int motor) {
    int i,first=motor>=0&&motor<3?motor:0,last=motor>=0&&motor<3?motor:2;
    for(i=first;i<=last;i++) {
        velocity_ramp_timestamp[i]=0;locked_rotor_since[i]=0;
        open_loop_timestamp[i]=0;shaft_velocity_sp[i]=0;
    }
}
static float velocity_coulomb_ff[3] = {0.12f, 0.45f, 0.12f};
static float velocity_bemf_ff[3] = {0.024f, 0.045f, 0.024f};

void velocity_feedforward_get(int motor, float *coulomb_v, float *bemf_v_per_rad_s)
{
	if(motor < 0 || motor > 2) return;
	if(coulomb_v) *coulomb_v = velocity_coulomb_ff[motor];
	if(bemf_v_per_rad_s) *bemf_v_per_rad_s = velocity_bemf_ff[motor];
}

void velocity_feedforward_set(int motor, float coulomb_v, float bemf_v_per_rad_s)
{
	int i;
	if(motor >= 0 && motor < 3)
	{
		velocity_coulomb_ff[motor] = coulomb_v;
		velocity_bemf_ff[motor] = bemf_v_per_rad_s;
	}
	else if(motor < 0)
	{
		for(i = 0; i < 3; i++)
		{
			velocity_coulomb_ff[i] = coulomb_v;
			velocity_bemf_ff[i] = bemf_v_per_rad_s;
		}
	}
}

static float ramp_velocity_target(float requested, int motor)
{
	uint32_t now_us = micros();
	float Ts = 0.001f;
	float max_delta;
	float delta;

	if(requested != requested) requested = 0.0f;
	requested = _constrain(requested, -velocity_limit, velocity_limit);
	if(velocity_ramp_timestamp[motor] != 0)
	{
		Ts = (float)(now_us - velocity_ramp_timestamp[motor]) * 1e-6f;
		if(Ts <= 0.0f || Ts > 0.02f) Ts = 0.02f;
	}
	velocity_ramp_timestamp[motor] = now_us;

	max_delta = (controller==Type_velocity_openloop?fminf(velocity_ramp_accel,100.0f):velocity_ramp_accel) * Ts;
	delta = requested - shaft_velocity_sp[motor];
	if(delta > max_delta) delta = max_delta;
	else if(delta < -max_delta) delta = -max_delta;
	shaft_velocity_sp[motor] += delta;
	return shaft_velocity_sp[motor];
}

/******************************************************************************/
void move(float new_target, int motor)
{
	if(motor < 0 || motor > 2) return;
    if((controller==Type_velocity || controller==Type_velocity_openloop) &&
       fabsf(new_target)>0.1f && fabsf(voltage[motor].q)>1.8f &&
       fabsf(shaft_velocity[motor])<0.1f) {
        uint32_t now=micros();
        if(!locked_rotor_since[motor]) locked_rotor_since[motor]=now;
        else if(now-locked_rotor_since[motor]>800000U) {Control_RequestStop(7);return;}
    } else locked_rotor_since[motor]=0;

	switch(controller)
	{
		case Type_velocity:
		{
			// 速度闭环控制模式 (FOC 矢量速度控制: 齿槽摩擦前馈 + 反电势解耦 + 抗饱和高动态 PID)
			loopFOC(motor);
			if(!motor_aligned[motor] || !sensor_valid[motor])
			{
                Control_RequestStop(6);
				pid_velocity[motor].integral_prev = 0.0f;
				pid_velocity[motor].error_prev = 0.0f;
				pid_velocity[motor].output_prev = 0.0f;
				pid_velocity[motor].timestamp_prev = 0;
				voltage[motor].q = 0.0f;
				voltage[motor].d = 0.0f;
				setPhaseVoltage(0.0f, 0.0f, electrical_angle[motor], motor);
				break;
			}
			float ramped_target = ramp_velocity_target(new_target, motor);

			// 仅在目标与实测速度都接近零时断输出，停车阶段保留反向力矩制动。
			if(fabsf(ramped_target) < 0.05f && fabsf(shaft_velocity[motor]) < 0.15f)
			{
				move_timestamp_prev[motor] = micros();
				pid_velocity[motor].integral_prev = 0.0f;
				pid_velocity[motor].error_prev = 0.0f;
				pid_velocity[motor].output_prev = 0.0f;
				voltage[motor].q = 0.0f;
				voltage[motor].d = 0.0f;
				setPhaseVoltage(0.0f, 0.0f, electrical_angle[motor], motor);
				break;
			}

			// 2. 计算控制周期 Ts
			uint32_t now_us = micros();
			float Ts;
			if(move_timestamp_prev[motor] == 0) Ts = 1e-3f;
			else Ts = (float)(now_us - (uint32_t)move_timestamp_prev[motor]) * 1e-6f;
			if(Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;
			move_timestamp_prev[motor] = now_us;

			float speed_err = ramped_target - shaft_velocity[motor];

			// 3. 极低速丝滑前馈 (Coulomb Friction & Detent Bias Feedforward):
			// 克服齿槽转矩死区卡顿，确保正向转动时推进力矩平滑向前，不在零点来回刹车跳变
			float uq_coulomb = velocity_coulomb_ff[motor] * tanhf(ramped_target / 0.5f);
			float uq_bemf = velocity_bemf_ff[motor] * ramped_target;
			float uq_ff = _constrain(uq_coulomb + uq_bemf, -voltage_limit, voltage_limit);

            /* Saturation/following error is not a stall: a rotating wheel may
               legitimately accelerate at full voltage. The locked-rotor trip
               above requires measured near-zero motion and stays latched. */
            float output_limit=voltage_limit;
			pid_velocity[motor].limit = output_limit;
			float uq_pid = PID_operator_offset(&pid_velocity[motor], speed_err, uq_ff);
			voltage[motor].q = _constrain(uq_ff + uq_pid, -output_limit, output_limit);
			voltage[motor].d = 0.0f;

			// 驱动相电压输出
			setPhaseVoltage(voltage[motor].q, voltage[motor].d, electrical_angle[motor], motor);
			break;
		}

		case Type_torque:
				// 力矩控制模式 (平衡控制器相电压输出模式)
				loopFOC(motor);
				if(!motor_aligned[motor] || !sensor_valid[motor])
				{
					voltage[motor].q = 0.0f;
					voltage[motor].d = 0.0f;
					setPhaseVoltage(0.0f, 0.0f, electrical_angle[motor], motor);
					break;
				}
				voltage[motor].q = _constrain(new_target, -voltage_limit, voltage_limit);
				voltage[motor].d = 0.0f;
				setPhaseVoltage(voltage[motor].q, voltage[motor].d, electrical_angle[motor], motor);
				break;

			case Type_velocity_openloop:
			// 速度开环模式 (同时读取传感器，保证上位机与遥测实时监测真实机械角度)
			updateSensor(motor);
			voltage[motor].q = velocityOpenloop(ramp_velocity_target(new_target, motor), motor);
			voltage[motor].d = 0.0f;
			break;

		case Type_angle_openloop:
			updateSensor(motor);
			shaft_angle_sp[motor] = new_target;
			voltage[motor].q = angleOpenloop(shaft_angle_sp[motor], motor);
			voltage[motor].d = 0.0f;
			break;

		default:
			break;
	}
}
/******************************************************************************/
/* 官方 SimpleFOC 标准 SpaceVectorPWM (Midpoint Clamp 鞍形波空间矢量调制)
 * 无扇区边界断点，全浮点硬件加速，相电压谐波最低，极其平滑连续
 */
void setPhaseVoltage(float Uq, float Ud, float angle_el, int motor)
{
    if(Control_Aborted() || !isfinite(Uq) || !isfinite(Ud) || !isfinite(angle_el) || voltage_power_supply<=0) {
        if(!Control_Aborted()) Control_RequestStop(6);
        return;
    }
	float _ca = cosf(angle_el);
	float _sa = sinf(angle_el);

	// 1. Inverse Park transform (反 Park 变换)
	float Ualpha = _ca * Ud - _sa * Uq;
	float Ubeta  = _sa * Ud + _ca * Uq;

	// 2. Clarke transform (Clarke 变换)
	float Ua = Ualpha;
	float Ub = -0.5f * Ualpha + 0.8660254f * Ubeta;
	float Uc = -0.5f * Ualpha - 0.8660254f * Ubeta;

	// 3. Space Vector PWM: 中点注入鞍形波 (Midpoint Clamp)
	float Umin = Ua < Ub ? (Ua < Uc ? Ua : Uc) : (Ub < Uc ? Ub : Uc);
	float Umax = Ua > Ub ? (Ua > Uc ? Ua : Uc) : (Ub > Uc ? Ub : Uc);
	float center = (voltage_power_supply * 0.5f) - (Umax + Umin) * 0.5f;

	Ua += center;
	Ub += center;
	Uc += center;

	// 4. 占空比归一化 [0.0, 1.0]
	float Ta = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
	float Tb = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
	float Tc = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

	switch(motor)
	{
		case 0:  /* M1: TIM1 通道1~3 (PE9, PE11, PE13) */
			TIM1->CCR1 = (uint32_t)(Ta * (PWM_Period_TIM1 + 1));
			TIM1->CCR2 = (uint32_t)(Tb * (PWM_Period_TIM1 + 1));
			TIM1->CCR3 = (uint32_t)(Tc * (PWM_Period_TIM1 + 1));
			break;
		case 1:  /* M2: TIM3 通道1~3 (PA6, PA7, PB0) */
			TIM3->CCR1 = (uint32_t)(Ta * (PWM_Period_APB1 + 1));
			TIM3->CCR2 = (uint32_t)(Tb * (PWM_Period_APB1 + 1));
			TIM3->CCR3 = (uint32_t)(Tc * (PWM_Period_APB1 + 1));
			break;
		case 2:  /* M3: TIM4 通道1~3 (PD12, PD13, PD14) */
			TIM4->CCR1 = (uint32_t)(Ta * (PWM_Period_APB1 + 1));
			TIM4->CCR2 = (uint32_t)(Tb * (PWM_Period_APB1 + 1));
			TIM4->CCR3 = (uint32_t)(Tc * (PWM_Period_APB1 + 1));
			break;
	}
}
/******************************************************************************/
/* 使用 TIM5 32位全硬件微秒时间戳 (1MHz) */
static unsigned long _micros(void)
{
	return (unsigned long)micros();
}

/******************************************************************************/
float velocityOpenloop(float target_velocity, int motor)
{
	unsigned long now_us = _micros();
	float Ts;

	if(open_loop_timestamp[motor] == 0)
	{
		Ts = 1e-3f;
	}
	else
	{
		Ts = (float)(now_us - open_loop_timestamp[motor]) * 1e-6f;
	}
	open_loop_timestamp[motor] = now_us;

	if(Ts <= 0.0f || Ts > 0.02f) Ts = 1e-3f;
	if(fabsf(target_velocity) < 0.05f)
	{
		setPhaseVoltage(0.0f, 0.0f, _normalizeAngle(open_loop_angle[motor] * (float)motor_pole_pairs[motor]), motor);
		return 0.0f;
	}

	// 专用独立开环角度累加器 (不污染 shaft_angle 多圈传感器真值)
	open_loop_angle[motor] = _normalizeAngle(open_loop_angle[motor] + target_velocity * Ts);

	// 电角度 = 机械角度 * 极对数
	setPhaseVoltage(voltage_limit, 0, _normalizeAngle(open_loop_angle[motor] * (float)motor_pole_pairs[motor]), motor);

	return voltage_limit;
}
/******************************************************************************/
float angleOpenloop(float target_angle, int motor)
{
	unsigned long now_us;
	float Ts,Uq;

	now_us = _micros();
	if(open_loop_timestamp[motor] == 0)
	{
		Ts = 1e-3f;
	}
	else
	{
		Ts = (float)(now_us - open_loop_timestamp[motor]) * 1e-6f;
	}
	open_loop_timestamp[motor] = now_us;

	if(Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

	if(fabsf(target_angle - shaft_angle[motor]) > velocity_limit * Ts)
	{
		shaft_angle[motor] += _sign(target_angle - shaft_angle[motor]) * velocity_limit * Ts;
	}
	else
	{
		shaft_angle[motor] = target_angle;
	}

	Uq = voltage_limit;
	setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle[motor], pole_pairs), motor);

	return Uq;
}
/******************************************************************************/




