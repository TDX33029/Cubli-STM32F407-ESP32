#include "lqr_balance.h"
#include "FOCMotor.h"
#include "BLDCMotor.h"
#include "timer.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
extern void set_motor_enable(uint8_t motor, uint8_t en);
extern uint8_t Control_Aborted(void);
extern void Control_RequestStop(uint8_t reason);
static uint32_t stall_since[3];
Cubli_Balance_t cubli_bal;
static uint8_t required_mask(void) {
    return cubli_bal.mode==BAL_MODE_CORNER?7U:(uint8_t)(1U<<cubli_bal.mode);
}
static bool corner_wheels_idle(void) {
    int i;
    if(cubli_bal.mode!=BAL_MODE_CORNER) return true;
    for(i=0;i<3;i++) if(!motor_aligned[i] || !sensor_valid[i] ||
        !isfinite(shaft_velocity[i]) || fabsf(shaft_velocity[i])>=5.0f) return false;
    return true;
}
bool LQR_AxesConfirmed(void) {return (cubli_bal.axes_confirmed_mask&required_mask())==required_mask();}
bool LQR_DirectionsConfirmed(void) {return (cubli_bal.directions_confirmed_mask&required_mask())==required_mask();}
void LQR_ConfirmAxis(uint8_t motor,uint8_t axis) {
    int i;uint8_t changed;
    if(motor>2 || axis>2) return;
    LQR_Balance_Enable(false);changed=(uint8_t)(1U<<motor);
    for(i=0;i<3;i++) if(i!=motor && cubli_bal.body_axis[i]==axis) {
        cubli_bal.body_axis[i]=cubli_bal.body_axis[motor];changed|=(uint8_t)(1U<<i);
    }
    cubli_bal.body_axis[motor]=axis;
    cubli_bal.axes_confirmed_mask=(cubli_bal.axes_confirmed_mask&~changed)|(1U<<motor);
    cubli_bal.directions_confirmed_mask&=~changed;
    memset(cubli_bal.reference_valid,0,sizeof(cubli_bal.reference_valid));
    LQR_ResetNeutralLearning();
}
void LQR_ConfirmDirection(uint8_t motor,int sign) {
    if(motor>2 || (sign!=1 && sign!=-1)) return;
    LQR_Balance_Enable(false);cubli_bal.body_direction[motor]=sign;
    cubli_bal.directions_confirmed_mask|=(uint8_t)(1U<<motor);
    LQR_ResetNeutralLearning();
}
void LQR_InvalidateMotorCalibration(uint8_t motor) {
    if(motor>2) return;
    LQR_Balance_Enable(false);
    cubli_bal.directions_confirmed_mask&=(uint8_t)~(1U<<motor);
    cubli_bal.reference_valid[motor]=false;
    cubli_bal.reference_valid[BAL_MODE_CORNER]=false;
    LQR_ResetNeutralLearning();
}
static float norm3(const float *v) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
static float dot3(const float *a, const float *b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
static void cross3(const float *a, const float *b, float *r) {
    r[0]=a[1]*b[2]-a[2]*b[1]; r[1]=a[2]*b[0]-a[0]*b[2]; r[2]=a[0]*b[1]-a[1]*b[0];
}
void LQR_ResetNeutralLearning(void) {
    cubli_bal.neutral_enabled=false;
    memset(cubli_bal.neutral_rotation,0,sizeof(cubli_bal.neutral_rotation));
    memset(cubli_bal.neutral_spin_filt,0,sizeof(cubli_bal.neutral_spin_filt));
    memcpy(cubli_bal.neutral_reference,cubli_bal.reference[3],sizeof(cubli_bal.neutral_reference));
}
bool LQR_SetNeutralLearning(bool enabled) {
    int i;
    if(cubli_bal.state!=BAL_STATE_DISABLED) return false;
    if(enabled) {
        if(cubli_bal.mode!=BAL_MODE_CORNER || !cubli_bal.reference_valid[3] ||
           !LQR_AxesConfirmed() || !LQR_DirectionsConfirmed() ||
           !cubli_bal.attitude.valid || !cubli_bal.attitude.is_calibrated ||
           HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30 || cubli_bal.K_tilt_trim!=0) return false;
        for(i=0;i<3;i++) if(cubli_bal.corner_gains[i].K_i!=0) return false;
    }
    if(!enabled) LQR_ResetNeutralLearning();
    cubli_bal.neutral_enabled=enabled;
    return true;
}
const float *LQR_GetControlReference(void) {
    return cubli_bal.mode==BAL_MODE_CORNER && cubli_bal.neutral_enabled?
           cubli_bal.neutral_reference:cubli_bal.reference[cubli_bal.mode];
}
bool LQR_SeedNeutralReference(const float gravity[3]) {
    int i;float seed[3],rotation[3],n,angle,s;
    const float *base=cubli_bal.reference[3];
    if(cubli_bal.state!=BAL_STATE_DISABLED || cubli_bal.mode!=BAL_MODE_CORNER ||
       !cubli_bal.neutral_enabled || !cubli_bal.reference_valid[3] ||
       !cubli_bal.attitude.valid || !cubli_bal.attitude.is_calibrated) return false;
    for(i=0;i<3;i++) if(!isfinite(gravity[i])) return false;
    n=norm3(gravity);if(n<.9f || n>1.1f) return false;
    for(i=0;i<3;i++) {seed[i]=gravity[i]/n;if(fabsf(seed[i])<.25f) return false;}
    cross3(seed,base,rotation);s=norm3(rotation);
    angle=atan2f(s,dot3(seed,base));
    if(angle>cubli_bal.neutral_limit_deg*DEG_TO_RAD+1e-6f) return false;
    for(i=0;i<3;i++) cubli_bal.neutral_rotation[i]=s>1e-8f?rotation[i]*angle/s:0;
    memcpy(cubli_bal.neutral_reference,seed,sizeof(seed));
    memset(cubli_bal.neutral_spin_filt,0,sizeof(cubli_bal.neutral_spin_filt));
    return true;
}
static void neutral_update(float dt) {
    int i;float spin[3]={0},projected[3],step[3],axis[3],rotated[3];
    float n,s,yaw,alpha,angle,c;
    const float *base=cubli_bal.reference[3];
    if(!cubli_bal.neutral_enabled || cubli_bal.mode!=BAL_MODE_CORNER ||
       cubli_bal.state!=BAL_STATE_BALANCING || dt<=0 || dt>0.03f) return;
    /* Freeze the slow loop while recovering or voltage-limited. Never use a
       release transient or invalid wheel measurement to move the target. */
    if(HAL_GetTick()-cubli_bal.balance_since_ms<300U ||
       HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30U ||
       norm3(cubli_bal.attitude.gyro_rad)>=0.5f || cubli_bal.K_tilt_trim!=0) goto freeze;
    cross3(cubli_bal.attitude.gravity,cubli_bal.neutral_reference,rotated);
    if(atan2f(norm3(rotated),dot3(cubli_bal.attitude.gravity,cubli_bal.neutral_reference))>5*DEG_TO_RAD) goto freeze;
    for(i=0;i<3;i++) {
        float limit=fminf(cubli_bal.corner_gains[i].max_u,voltage_limit);
        if(!motor_aligned[i] || !sensor_valid[i] || !isfinite(shaft_velocity[i]) ||
           cubli_bal.corner_gains[i].K_i!=0 || fabsf(cubli_bal.control_u[i])>=0.95f*limit) goto freeze;
        spin[cubli_bal.body_axis[i]]=-(float)cubli_bal.body_direction[i]*shaft_velocity[i];
    }
    /* Equal wheels: project their signed body momentum out of the vertical
       direction. Yaw spin cannot identify a different gravity reference. */
    yaw=dot3(spin,base);alpha=dt/(0.3f+dt);
    for(i=0;i<3;i++) {
        projected[i]=spin[i]-yaw*base[i];
        cubli_bal.neutral_spin_filt[i]+=alpha*(projected[i]-cubli_bal.neutral_spin_filt[i]);
    }
    n=norm3(cubli_bal.neutral_spin_filt);
    if(n<=3.0f) return;
    s=fminf(cubli_bal.neutral_gain*(n-3.0f),cubli_bal.neutral_rate_deg_s*DEG_TO_RAD)*dt/n;
    for(i=0;i<3;i++) step[i]=cubli_bal.neutral_rotation[i]+s*cubli_bal.neutral_spin_filt[i];
    n=norm3(step);
    if(n>cubli_bal.neutral_limit_deg*DEG_TO_RAD)
        for(i=0;i<3;i++) step[i]*=(cubli_bal.neutral_limit_deg*DEG_TO_RAD)/n;
    memcpy(cubli_bal.neutral_rotation,step,sizeof(step));
    angle=norm3(step);
    if(angle<1e-8f) {memcpy(cubli_bal.neutral_reference,base,sizeof(step));return;}
    for(i=0;i<3;i++) axis[i]=step[i]/angle;
    /* Reference follows the same body rotation convention as dg/dt=g x w. */
    cross3(base,axis,rotated);c=cosf(angle);s=sinf(angle);yaw=dot3(axis,base);
    for(i=0;i<3;i++) projected[i]=base[i]*c+rotated[i]*s+axis[i]*yaw*(1-c);
    n=norm3(projected);
    for(i=0;i<3;i++) cubli_bal.neutral_reference[i]=projected[i]/n;
    return;
freeze:
    memset(cubli_bal.neutral_spin_filt,0,sizeof(cubli_bal.neutral_spin_filt));
}
static void clear_output(void) {
    int i;
    cubli_bal.balance_since_ms=0;
    cubli_bal.full_speed_mask=0;
    memset(cubli_bal.neutral_spin_filt,0,sizeof(cubli_bal.neutral_spin_filt));
    set_motor_enable(0, 0);
    for(i=0;i<3;i++) {
        cubli_bal.control_u[i]=cubli_bal.wheel_int[i]=cubli_bal.tilt_int[i]=0;
        voltage[i].q=voltage[i].d=0;
        stall_since[i]=0;
    }
}
void LQR_Trip(void) {
    cubli_bal.auto_enabled=false;
    cubli_bal.stop_reason=BAL_STOP_FAULT;
    clear_output(); cubli_bal.state=BAL_STATE_FALL_PROTECT;
}
static void protect_balance(BalanceStop_e reason) {
    bool retry=cubli_bal.auto_enabled && (reason==BAL_STOP_ANGLE || reason==BAL_STOP_FULL_SPEED);
    clear_output();cubli_bal.stop_reason=reason;
    cubli_bal.capture_since_ms=0;
    cubli_bal.countdown_deadline_ms=cubli_bal.countdown_stable_ms=0;
    if(retry) {
        cubli_bal.auto_retry_deadline_ms=HAL_GetTick()+1000U;
        cubli_bal.state=BAL_STATE_WAIT_POSE;
    } else {
        cubli_bal.auto_enabled=false;
        cubli_bal.state=BAL_STATE_FALL_PROTECT;
    }
}
void LQR_Balance_Init(void) {
    int i;
    memset(&cubli_bal,0,sizeof(cubli_bal));
    cubli_bal.max_angle_trip=20*DEG_TO_RAD;
    cubli_bal.capture_angle=10*DEG_TO_RAD;
    cubli_bal.full_speed_limit=250;
    cubli_bal.K_yaw_damping=3.0f;
    cubli_bal.attitude_tau_s=0.25f;
    /* Experimental integral is opt-in; zero preserves the verified PD/wheel loop. */
    cubli_bal.K_tilt_trim=0.0f;
    cubli_bal.neutral_gain=0.000025f;
    cubli_bal.neutral_rate_deg_s=0.1f;
    cubli_bal.neutral_limit_deg=2;
    for(i=0;i<3;i++) {
        LQR_Gains_t g={32.0f,2.4f,0.08f,0.008f,0,5};
        cubli_bal.gains[i]=cubli_bal.corner_gains[i]=g;
        cubli_bal.body_direction[i]=1;
        cubli_bal.body_axis[i]=(uint8_t)i;
    }
    cubli_bal.reference[0][2]=cubli_bal.reference[1][2]=1;
    cubli_bal.reference[2][0]=1;
    for(i=0;i<3;i++) cubli_bal.reference[3][i]=0.577350269f;
}
void LQR_Balance_Enable(bool enable) {
    int i,first,last;
    clear_output(); cubli_bal.state=BAL_STATE_DISABLED;
    cubli_bal.auto_enabled=false;cubli_bal.auto_retry_deadline_ms=0;
    cubli_bal.capture_since_ms=0;
    cubli_bal.countdown_deadline_ms=cubli_bal.countdown_stable_ms=0;
    cubli_bal.countdown_seconds=0;
    if(!enable) return;
    if(!cubli_bal.reference_valid[cubli_bal.mode] || !LQR_AxesConfirmed() || !LQR_DirectionsConfirmed()) return;
    if(!cubli_bal.attitude.valid || !cubli_bal.attitude.is_calibrated ||
       HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30 || Control_Aborted()) return;
    first=cubli_bal.mode==BAL_MODE_CORNER?0:(int)cubli_bal.mode;
    last=cubli_bal.mode==BAL_MODE_CORNER?2:first;
    for(i=first;i<=last;i++) {
        updateSensor(i);
        if(!motor_aligned[i] || !sensor_valid[i]) return;
    }
    cubli_bal.last_update_us=micros();
    cubli_bal.state=BAL_STATE_STANDBY;
}
uint32_t LQR_CountdownRemainingMs(void) {
    int32_t remaining;
    if(cubli_bal.state!=BAL_STATE_COUNTDOWN) return 0;
    remaining=(int32_t)(cubli_bal.countdown_deadline_ms-HAL_GetTick());
    return remaining>0?(uint32_t)remaining:0;
}
bool LQR_StartCountdown(uint8_t seconds) {
    int i,first,last,m=(int)cubli_bal.mode;
    LQR_Balance_Enable(false);
    if(m>3 || seconds<2 || seconds>15 || !cubli_bal.reference_valid[m] || !LQR_AxesConfirmed() || !LQR_DirectionsConfirmed() ||
       !cubli_bal.attitude.valid || !cubli_bal.attitude.is_calibrated ||
       HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30 || Control_Aborted()) return false;
    first=m<3?m:0;last=m<3?m:2;
    for(i=first;i<=last;i++) {
        if(!motor_aligned[i]) return false;
        updateSensor(i);if(!sensor_valid[i] || Control_Aborted()) return false;
    }
    cubli_bal.last_update_us=micros();
    cubli_bal.countdown_seconds=seconds;
    cubli_bal.state=BAL_STATE_WAIT_POSE;
    return true;
}
bool LQR_StartAuto(uint8_t seconds) {
    if(cubli_bal.full_speed_limit>=velocity_limit || !LQR_StartCountdown(seconds)) return false;
    cubli_bal.auto_enabled=true;
    cubli_bal.max_angle_trip=20*DEG_TO_RAD;
    cubli_bal.stop_reason=BAL_STOP_NONE;
    return true;
}
uint32_t LQR_AutoRetryRemainingMs(void) {
    int32_t left;
    if(!cubli_bal.auto_enabled || !cubli_bal.auto_retry_deadline_ms) return 0;
    left=(int32_t)(cubli_bal.auto_retry_deadline_ms-HAL_GetTick());
    return left>0?(uint32_t)left:0;
}
bool LQR_AutoPoseReady(void) {
    int m=(int)cubli_bal.mode,i;
    float entry_deg=fminf(cubli_bal.capture_angle*RAD_TO_DEG,3.0f);
    if(m>3 || Control_Aborted() || LQR_AutoRetryRemainingMs() ||
       !LQR_AxesConfirmed() || !LQR_DirectionsConfirmed() || !cubli_bal.reference_valid[m] ||
       !cubli_bal.attitude.is_calibrated || !cubli_bal.attitude.valid ||
       HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30U ||
       cubli_bal.pose_tilt_deg>=entry_deg || fabsf(cubli_bal.signed_tilt_deg)>=entry_deg ||
       norm3(cubli_bal.attitude.gyro_rad)>=0.2f) return false;
    if(m<3) return motor_aligned[m] && sensor_valid[m] && fabsf(shaft_velocity[m])<5;
    for(i=0;i<3;i++) if(!motor_aligned[i] || !sensor_valid[i] || fabsf(shaft_velocity[i])>=5) return false;
    return true;
}
uint32_t LQR_FullSpeedElapsedMs(void) {
    int i;uint32_t longest=0;
    if(cubli_bal.state!=BAL_STATE_BALANCING) return 0;
    for(i=0;i<3;i++) if(cubli_bal.full_speed_mask&(1U<<i)) {
        uint32_t elapsed=HAL_GetTick()-cubli_bal.full_speed_since_ms[i];
        if(elapsed>longest) longest=elapsed;
    }
    return longest>3000U?3000U:longest;
}
bool LQR_StartProbe(float volts,uint16_t duration_ms) {
    int i,m=(int)cubli_bal.mode;
    if(cubli_bal.state!=BAL_STATE_DISABLED || m>2 || !LQR_AxesConfirmed() ||
       !motor_aligned[m] || !cubli_bal.attitude.valid || !cubli_bal.attitude.is_calibrated ||
       HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30 || Control_Aborted() ||
       !isfinite(volts) || fabsf(volts)>6 || fabsf(volts)>voltage_limit ||
       duration_ms<20 || duration_ms>120) return false;
    updateSensor(m);if(!sensor_valid[m] || fabsf(shaft_velocity[m])>5) return false;
    for(i=0;i<3;i++) cubli_bal.probe_start_gravity[i]=cubli_bal.attitude.gravity[i];
    cubli_bal.probe_voltage=volts;cubli_bal.probe_deadline_us=micros()+1000U*duration_ms;
    cubli_bal.probe_wheel_limit=(fabsf(volts)>3 && duration_ms>80)?100.0f:60.0f;
    cubli_bal.last_update_us=micros();
    set_motor_enable(m+1,1);if(Control_Aborted()) return false;
    cubli_bal.state=BAL_STATE_PROBE;return true;
}
void LQR_Balance_StartCalib(void) {
    cubli_bal.attitude_tau_s=0.25f;
    cubli_bal.neutral_limit_deg=2;
    LQR_ResetNeutralLearning();
    clear_output(); cubli_bal.state=BAL_STATE_CALIBRATING;
    cubli_bal.attitude.is_calibrated=false; cubli_bal.calib_samples=0;
    memset(cubli_bal.gyro_calib_sum,0,sizeof(cubli_bal.gyro_calib_sum));
}
void LQR_Attitude_Update(float a[3], float w[3], float dt) {
    int i;
    float n=norm3(a),pred[3],rot[3],alpha;
    Attitude_t *att=&cubli_bal.attitude;
    for(i=0;i<3;i++) if(!isfinite(a[i]) || !isfinite(w[i])) {att->valid=false;return;}
    if(n<0.2f || n>3.5f || dt<=0 || dt>0.03f) {att->valid=false;return;}
    if(cubli_bal.state==BAL_STATE_CALIBRATING) {
        if(fabsf(n-1)>0.15f || fabsf(w[0])>3 || fabsf(w[1])>3 || fabsf(w[2])>3) {
            cubli_bal.calib_samples=0;
            memset(cubli_bal.gyro_calib_sum,0,sizeof(cubli_bal.gyro_calib_sum)); return;
        }
        for(i=0;i<3;i++) cubli_bal.gyro_calib_sum[i]+=w[i]*DEG_TO_RAD;
        if(++cubli_bal.calib_samples==200) {
            for(i=0;i<3;i++) att->gyro_bias[i]=cubli_bal.gyro_calib_sum[i]/200;
            att->is_calibrated=true; att->valid=false;
            cubli_bal.state=BAL_STATE_DISABLED;
            printf("OK CAL_IMU:DONE\r\n");
        }
        return;
    }
    for(i=0;i<3;i++) att->gyro_rad[i]=w[i]*DEG_TO_RAD-att->gyro_bias[i];
    /* Body gravity propagation: dg/dt = g cross omega. No Euler singularity. */
    cross3(att->gravity,att->gyro_rad,rot);
    alpha=dt/(cubli_bal.attitude_tau_s+dt);
    if(fabsf(n-1)>0.25f) alpha=0;
    for(i=0;i<3;i++) {
        pred[i]=att->gravity[i]+rot[i]*dt;
        pred[i]=(1-alpha)*pred[i]+alpha*a[i]/n;
        if(!att->valid) pred[i]=a[i]/n;
        att->accel_filt[i]=a[i];
    }
    n=norm3(pred);
    for(i=0;i<3;i++) att->gravity[i]=pred[i]/n;
    att->pitch=atan2f(att->gravity[1],sqrtf(att->gravity[0]*att->gravity[0]+att->gravity[2]*att->gravity[2]));
    att->roll=atan2f(-att->gravity[0],att->gravity[2]);
    att->yaw+=att->gyro_rad[2]*dt;
    att->pitch_deg=att->pitch*RAD_TO_DEG; att->roll_deg=att->roll*RAD_TO_DEG;
    att->valid=true; att->last_sample_ms=HAL_GetTick();
}
bool LQR_CaptureReference(void) {
    int i,m=(int)cubli_bal.mode;
    if(!LQR_AxesConfirmed() || !LQR_DirectionsConfirmed()) return false;
    if(cubli_bal.state!=BAL_STATE_DISABLED || !cubli_bal.attitude.valid || !cubli_bal.attitude.is_calibrated ||
       norm3(cubli_bal.attitude.gyro_rad)>0.1f ||
       HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30) return false;
    if(m<3 && fabsf(cubli_bal.attitude.gravity[cubli_bal.body_axis[m]])>0.95f) return false;
    for(i=0;i<3;i++) cubli_bal.reference[m][i]=cubli_bal.attitude.gravity[i];
    if(m<3) cubli_bal.gains[m].theta_0=0;
    else for(i=0;i<3;i++) cubli_bal.corner_gains[i].theta_0=0;
    cubli_bal.reference_valid[m]=true;
    LQR_ResetNeutralLearning();
    return true;
}
void LQR_Balance_Loop(void) {
    int i,first,last,m=(int)cubli_bal.mode;
    uint32_t now=micros();
    float dt=(now-cubli_bal.last_update_us)*1e-6f,err[3]={0},tilt,full_tilt,rate_yaw,wheel_feedback_yaw=0;
    float *g=cubli_bal.attitude.gravity;
    const float *ref;
    cubli_bal.last_update_us=now;
    bool active=cubli_bal.state==BAL_STATE_STANDBY || cubli_bal.state==BAL_STATE_BALANCING ||
                cubli_bal.state==BAL_STATE_COUNTDOWN || cubli_bal.state==BAL_STATE_WAIT_POSE;
    if(cubli_bal.state==BAL_STATE_PROBE) {
        float probe_err[3];int axis=cubli_bal.body_axis[m];
        if(!cubli_bal.attitude.valid || HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30 ||
           dt<=0 || dt>0.03f || Control_Aborted() || norm3(cubli_bal.attitude.gyro_rad)>2) {LQR_Trip();return;}
        cross3(g,cubli_bal.probe_start_gravity,probe_err);
        cubli_bal.signed_tilt_deg=atan2f(probe_err[axis],dot3(g,cubli_bal.probe_start_gravity)-g[axis]*cubli_bal.probe_start_gravity[axis])*RAD_TO_DEG;
        cubli_bal.tilt_deg=fabsf(cubli_bal.signed_tilt_deg);
        cubli_bal.pose_tilt_deg=atan2f(norm3(probe_err),dot3(g,cubli_bal.probe_start_gravity))*RAD_TO_DEG;
        if(cubli_bal.pose_tilt_deg>10) {LQR_Trip();return;}
        return;
    }
    if(!cubli_bal.attitude.valid) {if(active) LQR_Trip();return;}
    neutral_update(dt);
    ref=LQR_GetControlReference();
    cross3(g,ref,err);
    full_tilt=atan2f(norm3(err),dot3(g,ref));
    cubli_bal.pose_tilt_deg=full_tilt*RAD_TO_DEG;
    if(cubli_bal.state==BAL_STATE_COUNTDOWN || cubli_bal.state==BAL_STATE_WAIT_POSE) {
        uint32_t ms=HAL_GetTick();int axis=m<3?cubli_bal.body_axis[m]:0;
        bool within=full_tilt<cubli_bal.capture_angle && (m==3 || fabsf(g[axis])<0.25f);
        if(!cubli_bal.attitude.valid || ms-cubli_bal.attitude.last_sample_ms>30 ||
           dt<=0 || dt>0.03f || Control_Aborted()) {LQR_Trip();return;}
        cubli_bal.tilt_deg=full_tilt*RAD_TO_DEG;
        cubli_bal.signed_tilt_deg=m<3?(atan2f(err[axis],dot3(g,ref)-g[axis]*ref[axis])-cubli_bal.gains[m].theta_0)*RAD_TO_DEG:full_tilt*RAD_TO_DEG;
        if(cubli_bal.auto_enabled) within=LQR_AutoPoseReady();
        within=within && corner_wheels_idle();
        if(cubli_bal.state==BAL_STATE_WAIT_POSE) {
            if(!within || norm3(cubli_bal.attitude.gyro_rad)>0.6f) {cubli_bal.capture_since_ms=0;return;}
            if(!cubli_bal.capture_since_ms) cubli_bal.capture_since_ms=ms;
            if(ms-cubli_bal.capture_since_ms>=(cubli_bal.auto_enabled?1000U:100U)) {
                cubli_bal.state=BAL_STATE_COUNTDOWN;
                cubli_bal.countdown_deadline_ms=ms+1000U*cubli_bal.countdown_seconds;
                cubli_bal.countdown_stable_ms=0;
            }
            return;
        }
        if(!within) {
            cubli_bal.state=BAL_STATE_WAIT_POSE;
            cubli_bal.countdown_deadline_ms=cubli_bal.countdown_stable_ms=cubli_bal.capture_since_ms=0;
            return;
        }
        if(norm3(cubli_bal.attitude.gyro_rad)<0.2f && full_tilt<fminf(cubli_bal.capture_angle,3*DEG_TO_RAD)) {
            if(!cubli_bal.countdown_stable_ms) cubli_bal.countdown_stable_ms=ms;
        } else cubli_bal.countdown_stable_ms=0;
        if((int32_t)(ms-cubli_bal.countdown_deadline_ms)>=0) {
            bool stable=cubli_bal.countdown_stable_ms && ms-cubli_bal.countdown_stable_ms>=500U;
            if(!stable) {
                cubli_bal.state=BAL_STATE_WAIT_POSE;
                cubli_bal.countdown_deadline_ms=cubli_bal.countdown_stable_ms=cubli_bal.capture_since_ms=0;
                return;
            }
            /* Keep the deliberately captured target; do not follow a moved pose. */
            bool automatic=cubli_bal.auto_enabled;
            uint8_t retry_seconds=cubli_bal.countdown_seconds;
            LQR_Balance_Enable(true);
            if(cubli_bal.state!=BAL_STATE_STANDBY) LQR_Trip();
            else {
                cubli_bal.auto_enabled=automatic;
                cubli_bal.countdown_seconds=retry_seconds;
                cubli_bal.capture_since_ms=ms-100U;
            }
        }
        return;
    }
    if(m<3) {
        int axis=cubli_bal.body_axis[m];
        float denom=dot3(g,ref)-g[axis]*ref[axis];
        if(1-ref[axis]*ref[axis]<0.05f || 1-g[axis]*g[axis]<0.05f) {if(active) LQR_Trip();return;}
        err[axis]=atan2f(err[axis],denom)-cubli_bal.gains[m].theta_0;
        cubli_bal.signed_tilt_deg=err[axis]*RAD_TO_DEG;
        tilt=fabsf(err[axis]);
    } else {
        float s=norm3(err);
        tilt=atan2f(s,dot3(g,ref));
        cubli_bal.signed_tilt_deg=tilt*RAD_TO_DEG;
        if(s>1e-6f) for(i=0;i<3;i++) err[i]*=tilt/s;
    }
    cubli_bal.tilt_deg=tilt*RAD_TO_DEG;
    if(!active) return;
    if(dt<=0 || dt>0.03f || HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30 || Control_Aborted()) {LQR_Trip();return;}
    /* Protect against falls away from the selected edge axis as well. */
    if(cubli_bal.state==BAL_STATE_BALANCING) {
        if(cubli_bal.neutral_enabled && m==3) {
            float base_err[3];
            cross3(g,cubli_bal.reference[3],base_err);
            if(atan2f(norm3(base_err),dot3(g,cubli_bal.reference[3]))>cubli_bal.max_angle_trip) {
                protect_balance(BAL_STOP_ANGLE);return;
            }
        }
        if(tilt>cubli_bal.max_angle_trip || full_tilt>cubli_bal.max_angle_trip) {protect_balance(BAL_STOP_ANGLE);return;}
        if(norm3(cubli_bal.attitude.gyro_rad)>3.0f) {protect_balance(BAL_STOP_BODY_RATE);return;}
    }
    if(cubli_bal.state==BAL_STATE_STANDBY) {
        uint32_t now_ms=HAL_GetTick();
        if(tilt>=fminf(cubli_bal.capture_angle,3*DEG_TO_RAD) || full_tilt>=fminf(cubli_bal.capture_angle,3*DEG_TO_RAD) || norm3(cubli_bal.attitude.gyro_rad)>0.35f || !corner_wheels_idle()) {
            cubli_bal.capture_since_ms=0;
            /* A pose lost between countdown expiry and enable needs a new countdown. */
            if(cubli_bal.countdown_seconds) {
                cubli_bal.state=BAL_STATE_WAIT_POSE;
                cubli_bal.countdown_deadline_ms=cubli_bal.countdown_stable_ms=0;
            }
            return;
        }
        if(cubli_bal.auto_enabled && !LQR_AutoPoseReady()) {cubli_bal.capture_since_ms=0;return;}
        if(!cubli_bal.capture_since_ms) {cubli_bal.capture_since_ms=now_ms;return;}
        if(now_ms-cubli_bal.capture_since_ms<100U) return;
        first=m<3?m:0; last=m<3?m:2;
        for(i=first;i<=last;i++) set_motor_enable(i+1,1);
        if(Control_Aborted()) {LQR_Trip();return;}
        cubli_bal.state=BAL_STATE_BALANCING;
        cubli_bal.balance_since_ms=HAL_GetTick();
        cubli_bal.stop_reason=BAL_STOP_NONE;cubli_bal.auto_retry_deadline_ms=0;
    }
    rate_yaw=dot3(cubli_bal.attitude.gyro_rad,ref);
    first=m<3?m:0; last=m<3?m:2;
    /* Corner wheel feedback stabilizes tilt only. Feeding its vertical
       component through the tilt Kw also drives yaw wheel momentum, which
       cannot identify a COM angle and can exhaust a wheel while tilt is small.
       BYAW remains the independent damping of rotation about gravity. */
    if(m==3) for(i=0;i<3;i++) {
        LQR_Gains_t *p=LQR_GetGains(i);
        if(!sensor_valid[i] || !motor_aligned[i] || !isfinite(shaft_velocity[i])) {LQR_Trip();return;}
        wheel_feedback_yaw+=(float)cubli_bal.body_direction[i]*ref[cubli_bal.body_axis[i]]*
                            (p->K_w*shaft_velocity[i]+p->K_i*cubli_bal.wheel_int[i]);
    }
    for(i=first;i<=last;i++) {
        LQR_Gains_t *p=LQR_GetGains(i);
        int axis=cubli_bal.body_axis[i];
        float rate=cubli_bal.attitude.gyro_rad[axis];
        float limit=fminf(p->max_u,voltage_limit),u,body;
        /* Corner handover starts with idle wheels and immediate release.
           Its stabilizing wheel term must be present from the first loop,
           rather than leaving a different controller active for500ms.
           Keep the established edge-mode handover ramp. */
        float wheel_weight=m==3?1.0f:_constrain(((float)(HAL_GetTick()-cubli_bal.balance_since_ms)-500.0f)/1000.0f,0,1);
        if(!sensor_valid[i] || !motor_aligned[i]) {LQR_Trip();return;}
        if(fabsf(shaft_velocity[i])>velocity_limit) {protect_balance(BAL_STOP_OVERSPEED);return;}
        if(fabsf(shaft_velocity[i])>=cubli_bal.full_speed_limit) {
            if(!(cubli_bal.full_speed_mask&(1U<<i))) {
                cubli_bal.full_speed_mask|=(uint8_t)(1U<<i);
                cubli_bal.full_speed_since_ms[i]=HAL_GetTick();
            } else if(HAL_GetTick()-cubli_bal.full_speed_since_ms[i]>=3000U) {protect_balance(BAL_STOP_FULL_SPEED);return;}
        } else cubli_bal.full_speed_mask&=(uint8_t)~(1U<<i);
        if(m==3) rate-=rate_yaw*ref[axis];
        body=p->K_theta*err[axis]+p->K_dtheta*rate;
        if(m==3) body+=cubli_bal.K_yaw_damping*rate_yaw*ref[axis];
        u=cubli_bal.body_direction[i]*body-wheel_weight*(p->K_w*shaft_velocity[i]+p->K_i*cubli_bal.wheel_int[i]);
        if(m==3) u+=(float)cubli_bal.body_direction[i]*wheel_feedback_yaw*ref[axis];
        /* Experimental attitude integral, in the same body-axis convention as PD.
           It integrates reference error; it does not identify the neutral COM pose. */
        if(m==3) u+=cubli_bal.body_direction[i]*wheel_weight*cubli_bal.K_tilt_trim*cubli_bal.tilt_int[i];
        if(wheel_weight>=1 && fabsf(u)<limit) {
            cubli_bal.wheel_int[i]=_constrain(cubli_bal.wheel_int[i]+shaft_velocity[i]*dt,-50,50);
            if(m==3 && cubli_bal.K_tilt_trim>0) cubli_bal.tilt_int[i]=_constrain(cubli_bal.tilt_int[i]+err[axis]*dt,-2,2);
        }
        if(!isfinite(u)) {LQR_Trip();return;}
        cubli_bal.control_u[i]=_constrain(u,-limit,limit);
    }
}
void LQR_Motor_Loop(void) {
    int i,m=(int)cubli_bal.mode,first=m<3?m:0,last=m<3?m:2;
    if(cubli_bal.state!=BAL_STATE_BALANCING && cubli_bal.state!=BAL_STATE_STANDBY && cubli_bal.state!=BAL_STATE_PROBE) return;
    if(Control_Aborted() || !cubli_bal.attitude.valid || HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30) {LQR_Trip();return;}
    if(cubli_bal.state==BAL_STATE_PROBE && (int32_t)(micros()-cubli_bal.probe_deadline_us)>=0) {
        clear_output();cubli_bal.state=BAL_STATE_DISABLED;return;
    }
    for(i=first;i<=last;i++) {
        loopFOC(i);
        if(!sensor_valid[i] || !motor_aligned[i]) {LQR_Trip();return;}
        if(cubli_bal.state==BAL_STATE_PROBE) {
            if(fabsf(shaft_velocity[i])>cubli_bal.probe_wheel_limit) {LQR_Trip();return;}
            voltage[i].q=_constrain(cubli_bal.probe_voltage,-voltage_limit,voltage_limit);voltage[i].d=0;
            cubli_bal.control_u[i]=voltage[i].q;
            setPhaseVoltage(voltage[i].q,0,electrical_angle[i],i);continue;
        }
        if(cubli_bal.state==BAL_STATE_BALANCING) {
            if(fabsf(cubli_bal.control_u[i])>1.8f && fabsf(shaft_velocity[i])<0.1f) {
                uint32_t now=micros();
                if(!stall_since[i]) stall_since[i]=now;
                else if(now-stall_since[i]>800000U) {Control_RequestStop(7);LQR_Trip();return;}
            } else stall_since[i]=0;
            voltage[i].q=cubli_bal.control_u[i]; voltage[i].d=0;
            setPhaseVoltage(voltage[i].q,0,electrical_angle[i],i);
        }
    }
}
void LQR_Balance_SetMode(BalanceMode_e mode) {
    LQR_Balance_Enable(false);
    cubli_bal.attitude_tau_s=0.25f;
    cubli_bal.neutral_limit_deg=2;
    LQR_ResetNeutralLearning();
    if(mode<=BAL_MODE_CORNER) {
        cubli_bal.mode=mode;
        cubli_bal.capture_angle=(mode==BAL_MODE_CORNER?0.5f:10.0f)*DEG_TO_RAD;
    }
}
LQR_Gains_t *LQR_GetGains(uint8_t idx) {
    if(idx>2) idx=0;
    return cubli_bal.mode==BAL_MODE_CORNER?&cubli_bal.corner_gains[idx]:&cubli_bal.gains[idx];
}
void LQR_SetGains(uint8_t idx,float kp,float kd,float kw,float ki) {
    LQR_Gains_t *g=LQR_GetGains(idx);
    if(cubli_bal.neutral_enabled && ki!=0) return;
    g->K_theta=kp;g->K_dtheta=kd;g->K_w=kw;g->K_i=ki;
}
void LQR_SetZeroAngle(uint8_t idx,float deg) {LQR_GetGains(idx)->theta_0=deg*DEG_TO_RAD;}
void LQR_SetMaxVoltage(float v) {
    int i; for(i=0;i<3;i++) cubli_bal.gains[i].max_u=cubli_bal.corner_gains[i].max_u=v;
}
void LQR_SetTiltTrim(float k) {if(!cubli_bal.neutral_enabled || k==0) cubli_bal.K_tilt_trim=k;}
bool LQR_SetAttitudeTau(float seconds) {
    if(cubli_bal.state!=BAL_STATE_DISABLED || !isfinite(seconds) || seconds<0.25f || seconds>2.0f) return false;
    cubli_bal.attitude_tau_s=seconds;return true;
}
