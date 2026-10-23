#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include "foc_utils.h"
/******************************************************************************/
/**
 *  Motiron control type
 */
typedef enum
{
	Type_torque,//!< Torque control
	Type_velocity,//!< Velocity motion control
	Type_angle,//!< Position/angle motion control
	Type_velocity_openloop,
	Type_angle_openloop
} MotionControlType;

/**
 *  Motiron control type
 */
typedef enum
{
	Type_voltage, //!< Torque control using voltage
	Type_dc_current, //!< Torque control using DC current (one current magnitude)
	Type_foc_current //!< torque control using dq currents
} TorqueControlType;

extern TorqueControlType torque_controller;
extern MotionControlType controller;
/******************************************************************************/
extern float shaft_angle[3];//!< current motor angle [0]=M1, [1]=M2, [2]=M3
extern float electrical_angle[3];
extern float shaft_velocity[3];
extern float current_sp;
extern float shaft_velocity_sp[3];
extern float shaft_angle_sp[3];
extern DQVoltage_s voltage[3];
extern DQCurrent_s current;

extern float sensor_offset[3];
extern float zero_electric_angle[3];
extern int   sensor_direction[3];   // 编码器旋转方向 (CW: 1, CCW: -1)
extern uint8_t motor_aligned[3];    // 电角度对齐完成标志 (1:已对齐, 0:未对齐)
extern uint8_t sensor_valid[3];     // 最近一次 MT6701 读取结果
extern uint8_t motor_sensor_map[3]; // 电机驱动通道到编码器传感器的实测映射
extern uint8_t motor_mapping_valid[3];
extern float angle_prev[3];         // 当前单圈绝对弧度 [0, 2PI)

extern PIDController_t pid_velocity[3]; // 速度环 PID
extern LowPassFilter_t lpf_velocity[3]; // 速度测量低通滤波器

extern float target_angle_el[3];        // 闭环同步磁场连续电角度
extern unsigned long move_timestamp_prev[3]; // 闭环控制更新微秒时间戳

/******************************************************************************/
void SimpleFOC_PID_Init(void);
void reset_closed_loop_state(int motor);
void reset_sensor_tracking(int motor);
float shaftAngle(int motor);
float shaftVelocity(int motor);
float electricalAngle(int motor);
void updateSensor(int motor);
typedef struct {uint32_t samples,errors; volatile uint32_t last_us; uint32_t max_gap_us,max_read_us;} EncoderDiagnostics_t;
extern EncoderDiagnostics_t encoder_diag[3];
extern uint32_t encoder_diag_epoch;
void reset_encoder_diagnostics(void);
/******************************************************************************/

#endif

