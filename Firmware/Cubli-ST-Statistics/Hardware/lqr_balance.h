#ifndef __LQR_BALANCE_H__
#define __LQR_BALANCE_H__
#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#define DEG_TO_RAD (3.141592653589793f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.141592653589793f)
typedef enum { BAL_STATE_DISABLED, BAL_STATE_CALIBRATING, BAL_STATE_STANDBY,
               BAL_STATE_BALANCING, BAL_STATE_FALL_PROTECT, BAL_STATE_COUNTDOWN,
               BAL_STATE_WAIT_POSE, BAL_STATE_PROBE } BalanceState_e;
typedef enum { BAL_MODE_EDGE_M1, BAL_MODE_EDGE_M2, BAL_MODE_EDGE_M3,
               BAL_MODE_CORNER } BalanceMode_e;
typedef enum { BAL_STOP_NONE, BAL_STOP_ANGLE, BAL_STOP_FULL_SPEED,
               BAL_STOP_OVERSPEED, BAL_STOP_BODY_RATE, BAL_STOP_FAULT } BalanceStop_e;
/* Voltage-domain state feedback; gains require identification on the real plant. */
typedef struct {
    float K_theta, K_dtheta, K_w, K_i, theta_0, max_u;
} LQR_Gains_t;
typedef struct {
    float pitch, roll, yaw, pitch_deg, roll_deg;
    float gyro_rad[3], accel_filt[3], gyro_bias[3], gravity[3];
    bool is_calibrated, valid;
    uint32_t last_sample_ms;
} Attitude_t;
typedef struct {
    BalanceState_e state;
    BalanceMode_e mode;
    LQR_Gains_t gains[3], corner_gains[3];
    float K_yaw_damping;
    float attitude_tau_s; /* Accelerometer correction time constant; default .25s. */
    Attitude_t attitude;
    float wheel_int[3], control_u[3];
    /* Per-motor control-law components (observation only, not used by the law):
       err in rad, P/D/wheel terms in volts. */
    float control_err[3], control_p[3], control_d[3], control_w[3];
    /* Experimental tilt-error integral, not a center-of-mass estimator. */
    float tilt_int[3], K_tilt_trim;
    /* Optional slow corner reference search. Original reference remains the
       safety anchor; STOP retains the bounded offset, configuration resets it. */
    bool neutral_enabled;
    float neutral_gain, neutral_rate_deg_s, neutral_limit_deg;
    float neutral_rotation[3], neutral_spin_filt[3], neutral_reference[3];
    /* M1/M2/M3 axes must be IMU body X/Y/Z; signs describe mechanical installation,
       independently of encoder sensor_direction. */
    int body_direction[3];
    uint8_t body_axis[3];
    uint8_t axes_confirmed_mask, directions_confirmed_mask;
    float reference[4][3], tilt_deg, pose_tilt_deg, signed_tilt_deg;
    uint32_t balance_since_ms;
    bool reference_valid[4];
    uint32_t capture_since_ms;
    uint32_t countdown_deadline_ms, countdown_stable_ms;
    uint8_t countdown_seconds;
    uint32_t probe_deadline_us;
    float probe_voltage, probe_wheel_limit, probe_start_gravity[3];
    float max_angle_trip, capture_angle;
    bool auto_enabled;
    BalanceStop_e stop_reason;
    float full_speed_limit;
    uint8_t full_speed_mask;
    uint32_t full_speed_since_ms[3], auto_retry_deadline_ms;
    uint32_t last_update_us, calib_samples;
    float gyro_calib_sum[3];
} Cubli_Balance_t;
extern Cubli_Balance_t cubli_bal;
void LQR_Balance_Init(void);
void LQR_Attitude_Update(float accel_g[3], float gyro_dps[3], float dt);
void LQR_Balance_Loop(void);       /* 200Hz state feedback only */
void LQR_Motor_Loop(void);         /* fast encoder commutation in main loop */
void LQR_Balance_Enable(bool enable);
bool LQR_StartCountdown(uint8_t seconds);
bool LQR_StartAuto(uint8_t seconds);
bool LQR_AutoPoseReady(void);
uint32_t LQR_AutoRetryRemainingMs(void);
uint32_t LQR_FullSpeedElapsedMs(void);
bool LQR_StartProbe(float voltage_v, uint16_t duration_ms);
uint32_t LQR_CountdownRemainingMs(void);
bool LQR_AxesConfirmed(void);
bool LQR_DirectionsConfirmed(void);
void LQR_ConfirmAxis(uint8_t motor, uint8_t axis);
void LQR_ConfirmDirection(uint8_t motor, int sign);
void LQR_InvalidateMotorCalibration(uint8_t motor);
void LQR_Balance_SetMode(BalanceMode_e mode);
void LQR_Balance_StartCalib(void); /* calibration NEVER arms motors */
bool LQR_CaptureReference(void);
void LQR_Trip(void);              /* latched; explicit BAL 1 required */
LQR_Gains_t *LQR_GetGains(uint8_t idx);
void LQR_SetGains(uint8_t idx, float kp, float kd, float kw, float ki);
void LQR_SetZeroAngle(uint8_t idx, float angle_deg);
void LQR_SetMaxVoltage(float max_v);
void LQR_SetTiltTrim(float k); /* corner tilt-error integral gain, 0..10 V/(rad*s) */
void LQR_ResetNeutralLearning(void);
bool LQR_SetNeutralLearning(bool enabled);
bool LQR_SeedNeutralReference(const float gravity[3]); /* disabled, enabled learner, inside configured cone */
const float *LQR_GetControlReference(void);
bool LQR_SetAttitudeTau(float seconds); /* disarm-only, .25..2s */
#endif
