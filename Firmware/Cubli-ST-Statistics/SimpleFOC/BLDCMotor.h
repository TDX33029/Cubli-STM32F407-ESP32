#ifndef BLDCMotor_H
#define BLDCMotor_H

/******************************************************************************/
/**
 *  Direction structure
 */
typedef enum
{
    CW      = 1,  //clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   //not yet known or invalid state
} Direction;

/******************************************************************************/
extern float voltage_power_supply;
extern float motor_test_travel_deg[3];
extern float voltage_limit;
extern float voltage_sensor_align;
extern int  pole_pairs;
#ifndef MOTOR_PP_M1
#define MOTOR_PP_M1 7
#endif
#ifndef MOTOR_PP_M2
#define MOTOR_PP_M2 7
#endif
#ifndef MOTOR_PP_M3
#define MOTOR_PP_M3 7
#endif
#ifndef OPENLOOP_VELOCITY_LIMIT
#define OPENLOOP_VELOCITY_LIMIT 10.0f
#endif
extern uint8_t motor_pole_pairs[3];
void reset_motion_state(int motor);
extern unsigned long open_loop_timestamp[3];
extern float velocity_limit;
extern float velocity_ramp_accel;
extern float current_limit;
/******************************************************************************/
void Motor_init(void);
void Motor_initFOC(void);
uint8_t Motor_alignSensor(int motor);
void Motor_alignAll(void);
void loopFOC(int motor);
void move(float new_target, int motor);
void velocity_feedforward_get(int motor, float *coulomb_v, float *bemf_v_per_rad_s);
void velocity_feedforward_set(int motor, float coulomb_v, float bemf_v_per_rad_s);
void setPhaseVoltage(float Uq, float Ud, float angle_el, int motor);
float velocityOpenloop(float target_velocity, int motor);
float angleOpenloop(float target_angle, int motor);
/******************************************************************************/

#endif
