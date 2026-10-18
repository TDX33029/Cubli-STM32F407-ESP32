/* Production control code with a PWM-to-encoder model at the hardware boundary. */
#include "mock_hw.h"
TIM_TypeDef mock_timers[3];GPIO_TypeDef mock_gpio;
static uint32_t fake_ms=100,fake_us=100000;
static const char *input_line;
static uint8_t encoder_ok=1,model_on=0;
static int settle_drift_reads;
static int16_t encoder_raw[3];
static int true_map[3]={2,0,1},true_sign[3]={1,-1,1},true_pp[3]={7,7,7};
static float rotor[3],field_previous[3];
static uint8_t field_seeded[3];
static uint16_t mock_tx_free=1023;
static unsigned uart_bytes;
uint32_t HAL_GetTick(void) {return fake_ms;}
uint32_t micros(void) {return fake_us;}
void delay_ms(unsigned short ms) {fake_ms+=ms;fake_us+=ms*1000;Control_Heartbeat();}
void delay_us(unsigned long us) {fake_us+=(uint32_t)us;fake_ms=fake_us/1000;Control_Heartbeat();}
uint8_t USART_ReadLine(char *line) {if(!input_line)return 0;strcpy(line,input_line);input_line=NULL;return 1;}
void USART_ClearCommands(void) {}
uint16_t USART_TxFree(void) {return mock_tx_free;}
void UART_WriteText(const char *text) {uart_bytes+=(unsigned)strlen(text);fputs(text,stdout);}
void UART_WriteUInt(uint32_t value) {uart_bytes+=(unsigned)printf("%u",value);}
void UART_WriteInt(int32_t value) {uart_bytes+=(unsigned)printf("%d",value);}
void UART_WriteFixed(float value,uint8_t decimals) {uart_bytes+=(unsigned)printf("%.*f",decimals,value);}
uint16_t ENC_CountsPerRev(uint8_t idx) {return idx<3?16384:0;}
static void model(void) {
    int m;
    extern volatile uint8_t m1_enabled,m2_enabled,m3_enabled;
    uint8_t flags[3]={m1_enabled,m2_enabled,m3_enabled};
    if(!model_on) return;
    for(m=0;m<3;m++) if(flags[m]) {
        TIM_TypeDef *t=&mock_timers[m];
        float alpha=t->CCR1-0.5f*(t->CCR2+t->CCR3),beta=0.8660254f*((float)t->CCR2-t->CCR3);
        float field=atan2f(beta,alpha),delta;
        if(alpha*alpha+beta*beta<1) continue;
        if(!field_seeded[m]) {field_previous[m]=field;field_seeded[m]=1;}
        delta=field-field_previous[m];if(delta>3.14159265f)delta-=6.2831853f;if(delta< -3.14159265f)delta+=6.2831853f;
        rotor[m]+=delta*true_sign[m]/true_pp[m];field_previous[m]=field;
        {
            float wrapped=fmodf(rotor[m],6.2831853f);if(wrapped<0)wrapped+=6.2831853f;
            encoder_raw[true_map[m]]=(int16_t)(wrapped*16384/6.2831853f);
        }
    }
}
uint8_t ENC_GetAngle(uint8_t idx,int16_t *raw,float *deg) {
    if(settle_drift_reads>0) {encoder_raw[idx]=(encoder_raw[idx]+8)%16384;settle_drift_reads--;}
    model();*raw=encoder_raw[idx];*deg=*raw*360.0f/16384;return !encoder_ok;
}
uint8_t LSM6DSR_Init(void) {return 0;}
uint8_t MPU6050_Init(void) {return 0;}
#include "control_under_test.c"
#include "utils_under_test.c"
#include "foc_under_test.c"
#include "motor_under_test.c"
#include "balance_under_test.c"
static void command(const char *s) {input_line=s;commander_run();}
static void advance(void) {fake_ms+=5;fake_us+=5000;main_alive_ms=fake_ms;Control_Heartbeat();}
static void gravity(float x,float y,float z) {
 cubli_bal.attitude.gravity[0]=x;cubli_bal.attitude.gravity[1]=y;cubli_bal.attitude.gravity[2]=z;
 cubli_bal.attitude.valid=true;cubli_bal.attitude.last_sample_ms=fake_ms;
}
static void setup(void) {
 int i;fake_ms=fake_us/1000;model_on=0;encoder_ok=1;settle_drift_reads=0;safety_trip=0;diagnostic_busy=0;
 mock_gpio.IDR=GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
 LQR_Balance_Init();SimpleFOC_PID_Init();voltage_power_supply=12;voltage_limit=5;velocity_limit=180;velocity_ramp_accel=400;pole_pairs=7;
 imu_online=1;Control_Heartbeat();main_alive_ms=fake_ms;controller=Type_velocity;
 command("APP 0");command("DBG 1");
 for(i=0;i<3;i++) {
  motor_pole_pairs[i]=7;motor_sensor_map[i]=(uint8_t)i;motor_mapping_valid[i]=1;motor_aligned[i]=sensor_valid[i]=1;
  sensor_direction[i]=1;encoder_raw[i]=0;rotor[i]=0;field_seeded[i]=0;
  reset_sensor_tracking(i);updateSensor(i);encoder_diag[i].last_us=fake_us;
 }
}
static void auto_ticks(int count,float angle,float wheel) {
 int j;
 for(j=0;j<count;j++) {
  advance();gravity(-sinf(angle),0,cosf(angle));shaft_velocity[0]=wheel;LQR_Balance_Loop();
  assert(!m2_enabled&&!m3_enabled);
 }
}
static void setup_auto(void) {
 setup();command("APP 1");command("BAXIS1 2");command("BDIR1 -1");command("L 330");
 cubli_bal.attitude.is_calibrated=true;gravity(0,0,1);command("BZERO");command("BFULL 250");command("BAUTO 1 3");
 assert(cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled);
}
static void test_auto_edge(void) {
 setup();command("APP 1");command("BAUTO 1");assert(!cubli_bal.auto_enabled&&!m1_enabled);
 setup_auto();command("BFULL 200");assert(cubli_bal.full_speed_limit==250);
 auto_ticks(300,4*DEG_TO_RAD,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 auto_ticks(100,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 cubli_bal.attitude.gyro_rad[1]=.21f;auto_ticks(220,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 cubli_bal.attitude.gyro_rad[1]=0;auto_ticks(220,0,6);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 auto_ticks(210,0,0);assert(cubli_bal.state==BAL_STATE_COUNTDOWN&&!m1_enabled);
 auto_ticks(1,4*DEG_TO_RAD,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled);
 auto_ticks(850,0,0);assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled&&cubli_bal.auto_enabled);
 auto_ticks(12000,0,0);assert(m1_enabled); /* No elapsed-run deadline, even after60s. */
 auto_ticks(1,19*DEG_TO_RAD,0);assert(m1_enabled);
 auto_ticks(1,21*DEG_TO_RAD,0);assert(!m1_enabled&&cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_WAIT_POSE&&cubli_bal.stop_reason==BAL_STOP_ANGLE);
 auto_ticks(250,0,40);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 auto_ticks(199,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 auto_ticks(2,0,0);assert(cubli_bal.state==BAL_STATE_COUNTDOWN);
 auto_ticks(610,0,0);assert(m1_enabled&&cubli_bal.auto_enabled);
 shaft_velocity[1]=300;auto_ticks(400,0,0);assert(m1_enabled&&!LQR_FullSpeedElapsedMs());shaft_velocity[1]=0;
 auto_ticks(400,0,250);assert(m1_enabled&&LQR_FullSpeedElapsedMs()<2000);
 auto_ticks(1,0,249);assert(!LQR_FullSpeedElapsedMs());
 auto_ticks(600,0,250);assert(m1_enabled&&LQR_FullSpeedElapsedMs()==2995);
 auto_ticks(1,0,250);assert(!m1_enabled&&cubli_bal.auto_enabled&&cubli_bal.stop_reason==BAL_STOP_FULL_SPEED);
 auto_ticks(1100,0,0);assert(m1_enabled&&cubli_bal.auto_enabled);
 auto_ticks(1,0,331);assert(!m1_enabled&&!cubli_bal.auto_enabled&&cubli_bal.stop_reason==BAL_STOP_OVERSPEED);
 auto_ticks(1000,0,0);assert(cubli_bal.state==BAL_STATE_FALL_PROTECT);
 setup_auto();auto_ticks(210,0,0);command("STOP");auto_ticks(1000,0,0);assert(!m1_enabled&&!cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_DISABLED);
 setup_auto();command("BAUTO 0");auto_ticks(1000,0,0);assert(!cubli_bal.auto_enabled&&!m1_enabled&&cubli_bal.state==BAL_STATE_DISABLED);
 setup_auto();fake_ms+=1501;fake_us+=1501000;main_alive_ms=fake_ms;Control_SafetyTick();gravity(0,0,1);LQR_Balance_Loop();assert(safety_trip==2&&!cubli_bal.auto_enabled&&!m1_enabled);
 setup_auto();cubli_bal.attitude.valid=false;LQR_Balance_Loop();assert(!cubli_bal.auto_enabled&&!m1_enabled&&cubli_bal.state==BAL_STATE_FALL_PROTECT);
 setup_auto();auto_ticks(850,0,0);fake_ms=0xffffffffU-1000U;Control_Heartbeat();gravity(0,0,1);
 auto_ticks(601,0,250);assert(!m1_enabled&&cubli_bal.stop_reason==BAL_STOP_FULL_SPEED); /* HAL tick rollover. */
 setup();mock_tx_free=899;uart_bytes=0;Control_Status();assert(!uart_bytes);
 mock_tx_free=1023;Control_Status();assert(uart_bytes>0&&uart_bytes<900);
}
static void corner_ticks(int count) {
 int i;
 for(i=0;i<count;i++) {
  advance();gravity(-.683937788f,.460044825f,.566204788f);LQR_Balance_Loop();
 }
}
static void test_corner_countdown(void) {
 int i;
 setup();command("APP 1");command("BAXIS 2 3 1");command("BDIR -1 -1 -1");command("BMODE 4");
 cubli_bal.attitude.is_calibrated=true;gravity(-.683937788f,.460044825f,.566204788f);
 assert(cubli_bal.K_yaw_damping==3);
 command("BYAW 2.5");assert(cubli_bal.K_yaw_damping==2.5f);
 command("BYAW 11");command("BYAW -1");command("BYAW nan");assert(cubli_bal.K_yaw_damping==2.5f);
 command("BYAW 3");
 cubli_bal.gains[0].theta_0=.7f;
 for(i=0;i<3;i++) cubli_bal.corner_gains[i].theta_0=.1f;
 command("BREF -0.683937788 0.460044825 0.566204788");
 assert(cubli_bal.reference_valid[3]&&cubli_bal.gains[0].theta_0==.7f);
 for(i=0;i<3;i++) assert(cubli_bal.corner_gains[i].theta_0==0);
 motor_aligned[2]=0;command("BSTART 3");assert(cubli_bal.state==BAL_STATE_DISABLED&&!m1_enabled&&!m2_enabled&&!m3_enabled);
 motor_aligned[2]=1;encoder_ok=0;command("BSTART 3");assert(cubli_bal.state==BAL_STATE_DISABLED&&!m1_enabled);
 encoder_ok=1;cubli_bal.directions_confirmed_mask=6;command("BSTART 3");assert(cubli_bal.state==BAL_STATE_DISABLED);
 cubli_bal.directions_confirmed_mask=7;command("BSTART 3");corner_ticks(30);
 assert(cubli_bal.state==BAL_STATE_COUNTDOWN&&!m1_enabled&&!m2_enabled&&!m3_enabled);
 advance();gravity(1,0,0);LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled);
 corner_ticks(650);assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled&&m2_enabled&&m3_enabled);
 command("BYAW 8");assert(cubli_bal.K_yaw_damping==3);
 for(i=0;i<3;i++) {
  cubli_bal.corner_gains[i].K_theta=cubli_bal.corner_gains[i].K_dtheta=0;
  cubli_bal.corner_gains[i].K_w=cubli_bal.corner_gains[i].K_i=0;
  cubli_bal.attitude.gyro_rad[i]=.2f*cubli_bal.reference[3][i];
 }
 corner_ticks(1);
 for(i=0;i<3;i++) assert(fabsf(cubli_bal.control_u[i]+.6f*cubli_bal.reference[3][cubli_bal.body_axis[i]])<.001f);
 cubli_bal.attitude.gyro_rad[0]=.1f;cubli_bal.attitude.gyro_rad[1]=.2f;cubli_bal.attitude.gyro_rad[2]=.2f;
 fast_mask=7;for(i=0;i<3;i++) {shaft_velocity[i]=-600;voltage[i].q=-12;}
 uart_bytes=0;Control_FastTelemetry();assert(uart_bytes>100&&uart_bytes<280);
 command("BREF -.577 .577 .577");assert(cubli_bal.reference_valid[3]&&fabsf(cubli_bal.reference[3][0]+.683937788f)<.0001f); /* Active BREF rejected. */
 advance();gravity(1,0,0);LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT&&!m1_enabled&&!m2_enabled&&!m3_enabled);
 command("BAUTO 1 3");assert(!cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_DISABLED&&!m1_enabled); /* L<=BFULL still rejects. */
 command("L 270");command("BAUTO 1 3");
 assert(cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled&&!m2_enabled&&!m3_enabled);
 command("BAUTO 0");assert(!cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_DISABLED);
}
static void corner_gravity(float angle) {
 float c=cosf(angle),s=sinf(angle),*p=cubli_bal.reference[3];
 /* Tilt the captured reference towards a fixed perpendicular direction. */
 gravity(p[0]*c+.5581f*s,p[1]*c+.8298f*s,p[2]*c);
}
static void corner_auto_ticks(int count,float angle,float w0,float w1,float w2) {
 int j;
 for(j=0;j<count;j++) {
  advance();corner_gravity(angle);
  shaft_velocity[0]=w0;shaft_velocity[1]=w1;shaft_velocity[2]=w2;
  LQR_Balance_Loop();
 }
}
static void setup_corner_auto(void) {
 setup();command("APP 1");command("BAXIS 2 3 1");command("BDIR -1 -1 -1");command("BMODE 4");
 cubli_bal.attitude.is_calibrated=true;corner_gravity(0);
 command("BREF -0.683937788 0.460044825 0.566204788");
 command("L 270");command("BFULL 250");command("BYAW 3");
 assert(cubli_bal.reference_valid[3]&&cubli_bal.full_speed_limit==250);
 command("BAUTO 1 3");
 assert(cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled&&!m2_enabled&&!m3_enabled);
}
static void test_corner_trim(void) {
 int i;
 float err[3];
 /* BTRIM is disarm-only and range-checked; off by default. */
 setup();
 assert(cubli_bal.K_tilt_trim==0.0f);
 command("BTRIM 3");assert(cubli_bal.K_tilt_trim==3.0f);
 command("BTRIM 11");command("BTRIM -1");command("BTRIM");assert(cubli_bal.K_tilt_trim==3.0f);
 command("BTRIM 2");assert(cubli_bal.K_tilt_trim==2.0f);
 /* Corner: a held 2deg tilt accumulates the integral and the trim term appears in u. */
 setup_corner_auto();
 cubli_bal.K_tilt_trim=2.0f; /* explicitly opt into the experimental controller */
 for(i=0;i<3;i++) {cubli_bal.corner_gains[i].K_theta=0;cubli_bal.corner_gains[i].K_dtheta=0;
                   cubli_bal.corner_gains[i].K_w=0;cubli_bal.corner_gains[i].K_i=0;}
 corner_auto_ticks(210,0,0,0,0);assert(cubli_bal.state==BAL_STATE_COUNTDOWN);
 corner_auto_ticks(910,0,0,0,0);assert(cubli_bal.state==BAL_STATE_BALANCING);
 corner_auto_ticks(320,0,0,0,0); /* remain at zero tilt before checking the integral */
 assert(cubli_bal.tilt_int[0]==0&&cubli_bal.tilt_int[1]==0&&cubli_bal.tilt_int[2]==0);
 corner_auto_ticks(200,2*DEG_TO_RAD,0,0,0);
 advance();corner_gravity(2*DEG_TO_RAD);cross3(cubli_bal.attitude.gravity,cubli_bal.reference[3],err);
 for(i=0;i<3;i++) {
  int axis=cubli_bal.body_axis[i];
  /* One second at 2deg: integral within the error magnitude; u carries -K_tilt_trim*integral. */
  assert(fabsf(cubli_bal.tilt_int[i])<=.045f&&signbit(cubli_bal.tilt_int[i])==signbit(err[axis]));
  assert(fabsf(cubli_bal.control_u[i]+2.0f*cubli_bal.tilt_int[i])<.001f);
 }
 /* Clamped at +/-2 rad*s; anti-windup freezes once u saturates. */
 cubli_bal.tilt_int[0]=1.99f;cubli_bal.tilt_int[1]=-1.99f;cubli_bal.tilt_int[2]=1.99f;
 corner_auto_ticks(400,6*DEG_TO_RAD,0,0,0);
 for(i=0;i<3;i++) assert(fabsf(cubli_bal.tilt_int[i])<=2.0001f);
 assert(fabsf(cubli_bal.tilt_int[2])==2.0f); /* M3 axis carries the positive error at +6deg */
 /* Integral polarity follows the confirmed body direction, including +1 installations. */
 cubli_bal.body_direction[0]=1;
 corner_auto_ticks(1,2*DEG_TO_RAD,0,0,0);
 assert(fabsf(cubli_bal.control_u[0]-2*cubli_bal.tilt_int[0])<.002f);
 /* Disarm clears the trim. */
 command("STOP");
 assert(cubli_bal.tilt_int[0]==0&&cubli_bal.tilt_int[1]==0&&cubli_bal.tilt_int[2]==0);
 /* Edge mode never accumulates the corner trim. */
 setup();command("APP 1");command("BAXIS1 2");command("BDIR1 -1");command("L 330");
 cubli_bal.attitude.is_calibrated=true;gravity(0,0,1);command("BZERO");command("BFULL 250");
 command("BAUTO 1 3");auto_ticks(850,2*DEG_TO_RAD,0);
 assert(cubli_bal.state==BAL_STATE_BALANCING);
 auto_ticks(600,2*DEG_TO_RAD,0);
 assert(cubli_bal.tilt_int[0]==0&&cubli_bal.tilt_int[1]==0&&cubli_bal.tilt_int[2]==0);
}
static void test_corner_verified_control_law(void) {
 int i;float err[3],s,angle,yaw;
 setup_corner_auto();
 for(i=0;i<3;i++) LQR_SetGains(i,80,5,-.030f,0);
 corner_auto_ticks(1150,0,0,0,0);
 corner_auto_ticks(400,0,0,0,0); /* full wheel feedback weight */
 cubli_bal.attitude.gyro_rad[0]=.01f;cubli_bal.attitude.gyro_rad[1]=-.02f;cubli_bal.attitude.gyro_rad[2]=.03f;
 corner_auto_ticks(100,2*DEG_TO_RAD,5,-8,11);
 cross3(cubli_bal.attitude.gravity,cubli_bal.reference[3],err);
 s=norm3(err);angle=atan2f(s,dot3(cubli_bal.attitude.gravity,cubli_bal.reference[3]));
 yaw=dot3(cubli_bal.attitude.gyro_rad,cubli_bal.reference[3]);
 for(i=0;i<3;i++) {
  int axis=cubli_bal.body_axis[i];float r=cubli_bal.reference[3][axis];
  float expected=-(80*err[axis]*angle/s+5*(cubli_bal.attitude.gyro_rad[axis]-yaw*r)+3*yaw*r)+.030f*shaft_velocity[i];
  assert(fabsf(cubli_bal.control_u[i]-expected)<.0001f);
  assert(cubli_bal.tilt_int[i]==0); /* no hidden integral with BTRIM off */
 }
}
static void test_corner_immediate_feedback(void) {
 int i;
 setup_corner_auto();command("BAUTO 0");
 for(i=0;i<3;i++) LQR_SetGains(i,0,0,-.040f,0);
 command("BSTART 2");
 corner_auto_ticks(100,0,0,0,6);
 assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled&&!m2_enabled&&!m3_enabled);
 corner_auto_ticks(30,0,2,0,0);assert(cubli_bal.state==BAL_STATE_COUNTDOWN);
 corner_auto_ticks(1,0,2,6,0);
 assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled);
 for(i=0;i<500&&cubli_bal.state!=BAL_STATE_STANDBY;i++) corner_auto_ticks(1,0,2,0,0);
 assert(cubli_bal.state==BAL_STATE_STANDBY&&!m1_enabled);
 /* A wheel starting to move in the handover gap must restart entry. */
 corner_auto_ticks(1,0,6,0,0);
 assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled);
 for(i=0;i<500&&cubli_bal.state!=BAL_STATE_BALANCING;i++) corner_auto_ticks(1,0,2,0,0);
 assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled&&m2_enabled&&m3_enabled);
 corner_auto_ticks(1,0,2,0,0);
 assert(HAL_GetTick()-cubli_bal.balance_since_ms<20U);
 assert(fabsf(cubli_bal.control_u[0]-.080f)<.0001f);
 command("STOP");
}
static void test_corner_auto(void) {
 /* Arming needs all three axes, sensors, directions and a reference. */
 setup_corner_auto();
 command("BAUTO 0");
 cubli_bal.directions_confirmed_mask=6;command("BAUTO 1 3");
 assert(!cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_DISABLED);
 cubli_bal.directions_confirmed_mask=7;
 motor_aligned[1]=0;command("BAUTO 1 3");assert(!cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_DISABLED);
 motor_aligned[1]=1;encoder_ok=0;command("BAUTO 1 3");assert(!cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_DISABLED);
 encoder_ok=1;command("BAUTO 1 3");
 assert(cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_WAIT_POSE);
 /* Entry needs <3deg, low body rate and all three wheels below5rad/s. */
 corner_auto_ticks(220,4*DEG_TO_RAD,0,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 corner_auto_ticks(100,0,0,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 cubli_bal.attitude.gyro_rad[0]=.21f;corner_auto_ticks(220,0,0,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 cubli_bal.attitude.gyro_rad[0]=0;corner_auto_ticks(220,0,0,0,6);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 corner_auto_ticks(210,0,0,0,0);
 assert(cubli_bal.state==BAL_STATE_COUNTDOWN&&!m1_enabled&&!m2_enabled&&!m3_enabled);
 corner_auto_ticks(1,4*DEG_TO_RAD,0,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled);
 corner_auto_ticks(900,0,0,0,0);
 assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled&&m2_enabled&&m3_enabled&&cubli_bal.auto_enabled);
 corner_auto_ticks(12000,0,0,0,0);assert(m1_enabled&&m2_enabled&&m3_enabled); /* No run deadline, even after60s. */
 corner_auto_ticks(1,19*DEG_TO_RAD,0,0,0);assert(m1_enabled&&m2_enabled&&m3_enabled);
 corner_auto_ticks(1,21*DEG_TO_RAD,0,0,0);
 assert(!m1_enabled&&!m2_enabled&&!m3_enabled&&cubli_bal.auto_enabled&&
        cubli_bal.state==BAL_STATE_WAIT_POSE&&cubli_bal.stop_reason==BAL_STOP_ANGLE);
 corner_auto_ticks(250,0,0,0,40);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 corner_auto_ticks(199,0,0,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 corner_auto_ticks(2,0,0,0,0);assert(cubli_bal.state==BAL_STATE_COUNTDOWN);
 corner_auto_ticks(610,0,0,0,0);assert(m1_enabled&&m2_enabled&&m3_enabled&&cubli_bal.auto_enabled);
 /* Full speed on the third axis is protected and retried like the edges. */
 corner_auto_ticks(400,0,0,0,250);assert(m1_enabled&&LQR_FullSpeedElapsedMs()<2000);
 corner_auto_ticks(1,0,0,0,249);assert(!LQR_FullSpeedElapsedMs());
 corner_auto_ticks(600,0,0,0,250);assert(m1_enabled&&LQR_FullSpeedElapsedMs()==2995);
 corner_auto_ticks(1,0,0,0,250);
 assert(!m1_enabled&&cubli_bal.auto_enabled&&cubli_bal.stop_reason==BAL_STOP_FULL_SPEED);
 corner_auto_ticks(1100,0,0,0,0);assert(m1_enabled&&cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_BALANCING);
 /* Overspeed exits automatic mode for good. */
 corner_auto_ticks(1,0,271,0,0);
 assert(!m1_enabled&&!m2_enabled&&!m3_enabled&&!cubli_bal.auto_enabled&&cubli_bal.stop_reason==BAL_STOP_OVERSPEED);
 corner_auto_ticks(1500,0,0,0,0);assert(cubli_bal.state==BAL_STATE_FALL_PROTECT);
 setup_corner_auto();corner_auto_ticks(220,0,0,0,0);command("STOP");corner_auto_ticks(1000,0,0,0,0);
 assert(!m1_enabled&&!cubli_bal.auto_enabled&&cubli_bal.state==BAL_STATE_DISABLED);
 setup_corner_auto();command("BAUTO 0");corner_auto_ticks(1000,0,0,0,0);
 assert(!cubli_bal.auto_enabled&&!m1_enabled&&!m2_enabled&&!m3_enabled&&cubli_bal.state==BAL_STATE_DISABLED);
}
static void test_corner_tight_entry(void) {
 int i;uint32_t begin;
 setup_corner_auto();command("BAUTO 0");
 assert(fabsf(cubli_bal.capture_angle-0.5f*DEG_TO_RAD)<.00001f);
 command("BCAP 0.49");assert(fabsf(cubli_bal.capture_angle-0.5f*DEG_TO_RAD)<.00001f);
 command("BCAP 0.5");
 command("BSTART 1");assert(cubli_bal.state==BAL_STATE_DISABLED);
 command("BSTART 2.5");assert(cubli_bal.state==BAL_STATE_DISABLED);
 command("BSTART 2");assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 corner_auto_ticks(250,.6f*DEG_TO_RAD,0,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled);
 for(i=0;i<30 && cubli_bal.state==BAL_STATE_WAIT_POSE;i++) corner_auto_ticks(1,.1f*DEG_TO_RAD,0,0,0);
 assert(cubli_bal.state==BAL_STATE_COUNTDOWN&&LQR_CountdownRemainingMs()==2000);
 corner_auto_ticks(100,.1f*DEG_TO_RAD,0,0,0);
 corner_auto_ticks(1,.6f*DEG_TO_RAD,0,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled);
 for(i=0;i<30 && cubli_bal.state==BAL_STATE_WAIT_POSE;i++) corner_auto_ticks(1,.1f*DEG_TO_RAD,0,0,0);
 begin=fake_ms;assert(LQR_CountdownRemainingMs()==2000);
 corner_auto_ticks(399,.1f*DEG_TO_RAD,0,0,0);assert(LQR_CountdownRemainingMs()==5&&!m1_enabled);
 corner_auto_ticks(1,.1f*DEG_TO_RAD,0,0,0);
 assert(fake_ms-begin==2000&&cubli_bal.state==BAL_STATE_STANDBY&&!m1_enabled);
 /* A late exit must restart the countdown, not enable on the next return. */
 corner_auto_ticks(1,.6f*DEG_TO_RAD,0,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled);
 corner_auto_ticks(425,.1f*DEG_TO_RAD,0,0,0);assert(m1_enabled&&m2_enabled&&m3_enabled);
 command("STOP");command("BAUTO 1"); /* Corner default countdown is2s. */
 assert(cubli_bal.auto_enabled&&cubli_bal.countdown_seconds==2);
 corner_auto_ticks(250,.6f*DEG_TO_RAD,0,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE);
 for(i=0;i<250 && cubli_bal.state==BAL_STATE_WAIT_POSE;i++) corner_auto_ticks(1,.1f*DEG_TO_RAD,0,0,0);
 assert(cubli_bal.state==BAL_STATE_COUNTDOWN&&LQR_CountdownRemainingMs()==2000);
 corner_auto_ticks(1,.6f*DEG_TO_RAD,0,0,0);assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled);
 command("STOP");
}
static void setup_neutral(void) {
 int i;
 setup_corner_auto();command("BAUTO 0");command("U 6");command("BU 6");
 for(i=0;i<3;i++) LQR_SetGains(i,0,0,0,0);
 command("BNEUTRAL 1 .00025 .5");assert(cubli_bal.neutral_enabled);
 command("BSTART 2");corner_auto_ticks(500,0,0,0,0);
 assert(cubli_bal.state==BAL_STATE_BALANCING);
}
static void neutral_tick(float w0,float w1,float w2) {
 const float *r=LQR_GetControlReference();float g[3];
 memcpy(g,r,sizeof(g));advance();gravity(g[0],g[1],g[2]);
 shaft_velocity[0]=w0;shaft_velocity[1]=w1;shaft_velocity[2]=w2;
 LQR_Balance_Loop();
}
static void test_neutral_search(void) {
 int i;float saved[3],previous[3],positive[3],*base,n;
 setup();assert(!cubli_bal.neutral_enabled);
 assert(cubli_bal.neutral_gain==.000025f && cubli_bal.neutral_rate_deg_s==.1f);
 command("BNEUTRAL 1");assert(!cubli_bal.neutral_enabled);
 setup_corner_auto();command("BAUTO 0");
 command("BNEUTRAL 1");assert(!cubli_bal.neutral_enabled); /* default Ki conflicts */
 for(i=0;i<3;i++) LQR_SetGains(i,0,0,0,0);
 cubli_bal.attitude.is_calibrated=false;command("BNEUTRAL 1");assert(!cubli_bal.neutral_enabled);
 cubli_bal.attitude.is_calibrated=true;command("BTRIM 1");command("BNEUTRAL 1");assert(!cubli_bal.neutral_enabled);
 command("BTRIM 0");command("BNEUTRAL 1");assert(cubli_bal.neutral_enabled);
 command("BNEUTRAL 1 .000251 .1");command("BNEUTRAL 1 .000004 .1");command("BNEUTRAL 1 .000025 .51");
 assert(cubli_bal.neutral_gain==.000025f && cubli_bal.neutral_rate_deg_s==.1f);
 command("BNEUTRAL 1 .00001 .05");assert(cubli_bal.neutral_gain==.00001f && cubli_bal.neutral_rate_deg_s==.05f);
 command("LQR1 80 8 -.04 .01");assert(cubli_bal.corner_gains[0].K_i==0);
 command("BTRIM 1");assert(cubli_bal.K_tilt_trim==0);
 setup_neutral();command("BNEUTRAL 0");assert(cubli_bal.neutral_enabled); /* disarm-only */
 /* Release transient does not move the target. */
 neutral_tick(200,0,0);assert(norm3(cubli_bal.neutral_rotation)==0);
 for(i=0;i<1200;i++) {
  memcpy(previous,cubli_bal.neutral_rotation,sizeof(previous));neutral_tick(200,0,0);
  for(int j=0;j<3;j++) saved[j]=cubli_bal.neutral_rotation[j]-previous[j];
  assert(norm3(saved)<=.5f*DEG_TO_RAD*.005f+1e-7f);
  assert(norm3(cubli_bal.neutral_rotation)<=2*DEG_TO_RAD+1e-7f);
  assert(fabsf(norm3(LQR_GetControlReference())-1)<1e-6f);
 }
 assert(fabsf(norm3(cubli_bal.neutral_rotation)-2*DEG_TO_RAD)<1e-6f);
 memcpy(positive,cubli_bal.neutral_rotation,sizeof(positive));
 assert(positive[1]>0); /* M1/IMU Y, direction -1 */
 base=cubli_bal.reference[3];assert(fabsf(dot3(positive,base))<1e-6f);
 /* Freeze on body rate, saturation, or excessive tracking error. */
 memcpy(saved,positive,sizeof(saved));cubli_bal.attitude.gyro_rad[0]=.6f;
 neutral_tick(-200,0,0);assert(memcmp(saved,cubli_bal.neutral_rotation,sizeof(saved))==0);
 cubli_bal.attitude.gyro_rad[0]=0;cubli_bal.control_u[0]=5.8f;
 neutral_tick(-200,0,0);assert(memcmp(saved,cubli_bal.neutral_rotation,sizeof(saved))==0);
 advance();gravity(1,0,0);LQR_Balance_Loop();assert(memcmp(saved,cubli_bal.neutral_rotation,sizeof(saved))==0);
 assert(!m1_enabled&&!m2_enabled&&!m3_enabled);
 /* STOP retains learning; BREF and configuration changes reset it. */
 command("STOP");assert(cubli_bal.neutral_enabled&&memcmp(saved,cubli_bal.neutral_rotation,sizeof(saved))==0);
 command("BREF -.683937788 .460044825 .566204788");assert(!cubli_bal.neutral_enabled&&norm3(cubli_bal.neutral_rotation)==0);
 setup_neutral();for(i=0;i<100;i++) neutral_tick(100,0,0);
 memcpy(saved,cubli_bal.neutral_rotation,sizeof(saved));sensor_valid[0]=false;
 neutral_tick(100,0,0);assert(memcmp(saved,cubli_bal.neutral_rotation,sizeof(saved))==0&&!m1_enabled);
 setup_neutral();for(i=0;i<100;i++) neutral_tick(100,0,0);
 memcpy(saved,cubli_bal.neutral_rotation,sizeof(saved));advance();cubli_bal.attitude.last_sample_ms=fake_ms-31;
 LQR_Balance_Loop();assert(memcmp(saved,cubli_bal.neutral_rotation,sizeof(saved))==0&&!m1_enabled);
 setup_neutral();for(i=0;i<300;i++) neutral_tick(-100,0,0);
 assert(dot3(cubli_bal.neutral_rotation,positive)<0);
 setup_neutral();cubli_bal.body_direction[0]=1;
 for(i=0;i<300;i++) neutral_tick(100,0,0);
 assert(dot3(cubli_bal.neutral_rotation,positive)<0);
 /* Vertical signed wheel spin alone cannot move the tilt target. */
 setup_neutral();base=cubli_bal.reference[3];
 for(i=0;i<1000;i++) neutral_tick(100*base[1],100*base[2],100*base[0]);
 assert(norm3(cubli_bal.neutral_rotation)<1e-7f);
 /* Once filtered spin has decayed inside the deadband the target holds. */
 for(i=0;i<100;i++) neutral_tick(100,0,0);
 for(i=0;i<400;i++) neutral_tick(0,0,0);
 memcpy(saved,cubli_bal.neutral_rotation,sizeof(saved));
 for(i=0;i<200;i++) neutral_tick(0,0,0);
 assert(memcmp(saved,cubli_bal.neutral_rotation,sizeof(saved))==0);
 /* A learned 2deg target outside the 0.5deg base entry is allowed on retry. */
 for(i=0;i<1500;i++) neutral_tick(200,0,0);
 memcpy(saved,LQR_GetControlReference(),sizeof(saved));command("BAL 0");command("BCAP .5");command("BAUTO 1 2");
 for(i=0;i<850;i++) {advance();gravity(saved[0],saved[1],saved[2]);shaft_velocity[0]=shaft_velocity[1]=shaft_velocity[2]=0;LQR_Balance_Loop();}
 assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled);
 /* Base fall protection cannot be moved by the learned 2deg target. */
 n=norm3(cubli_bal.neutral_rotation);
 for(i=0;i<3;i++) previous[i]=cubli_bal.neutral_rotation[i]/n;
 cross3(base,previous,saved);
 for(i=0;i<3;i++) saved[i]=base[i]*cosf(21*DEG_TO_RAD)+saved[i]*sinf(21*DEG_TO_RAD);
 advance();gravity(saved[0],saved[1],saved[2]);LQR_Balance_Loop();
 assert(cubli_bal.stop_reason==BAL_STOP_ANGLE&&!m1_enabled);
 /* Production packet budgets, with large signed values and countdown field. */
 setup_neutral();fast_mask=7;cubli_bal.state=BAL_STATE_COUNTDOWN;
 cubli_bal.countdown_deadline_ms=fake_ms+15000;
 for(i=0;i<3;i++) {shaft_velocity[i]=-600;shaft_velocity_sp[i]=-600;voltage[i].q=-12;cubli_bal.attitude.gyro_rad[i]=-3;}
 cubli_bal.tilt_deg=cubli_bal.signed_tilt_deg=180;
 uart_bytes=0;Control_FastTelemetry();assert(uart_bytes<280);
 uart_bytes=0;Control_Status();assert(uart_bytes<900);
 command("BAL 0");command("BMODE 1");assert(!cubli_bal.neutral_enabled);
 setup_neutral();command("BAL 0");command("CAL_IMU");assert(!cubli_bal.neutral_enabled);
 setup_neutral();command("BAL 0");command("BAXIS 1 2 3");assert(!cubli_bal.neutral_enabled);
 setup_neutral();command("BAL 0");command("BDIR 1 1 1");assert(!cubli_bal.neutral_enabled);
 setup_neutral();command("BAL 0");LQR_InvalidateMotorCalibration(0);assert(!cubli_bal.neutral_enabled);
}
static void test_neutral_five_degree_seed(void) {
 int i;float saved[3],before[3],axis[3],fallen[3],step[3],n;
 const float *base;
 setup_neutral();command("BAL 0");
 assert(cubli_bal.neutral_limit_deg==2);
 command("BNEUTRAL 1 .00025 .5 5");assert(cubli_bal.neutral_limit_deg==5);
 command("BNEUTRAL 1 .00025 .5 5.01");assert(cubli_bal.neutral_limit_deg==5);
 command("BNEUTRAL 1 .00025 .5 1.99");assert(cubli_bal.neutral_limit_deg==5);
 /* Serial seed uses the original fixed reference, not a new safety anchor. */
 base=cubli_bal.reference[3];memcpy(saved,base,sizeof(saved));
 command("BNEUTRAL 1 .00025 .5 5 31.39 50.38");
 assert(cubli_bal.neutral_enabled&&norm3(cubli_bal.neutral_rotation)>2*DEG_TO_RAD);
 assert(memcmp(saved,base,sizeof(saved))==0);
 assert(fabsf(norm3(LQR_GetControlReference())-1)<1e-6f);
 memcpy(before,LQR_GetControlReference(),sizeof(before));
 command("BNEUTRAL 1 .00025 .5 2");assert(cubli_bal.neutral_limit_deg==5);
 assert(memcmp(before,LQR_GetControlReference(),sizeof(before))==0);
 /* Seed log/exp convention must retain its direction on the first update. */
 command("BSTART 2");
 for(i=0;i<850;i++) {advance();gravity(before[0],before[1],before[2]);shaft_velocity[0]=shaft_velocity[1]=shaft_velocity[2]=0;LQR_Balance_Loop();}
 assert(cubli_bal.state==BAL_STATE_BALANCING);
 neutral_tick(0,0,0);
 for(i=0;i<3;i++) assert(fabsf(LQR_GetControlReference()[i]-before[i])<1e-6f);
 command("BNEUTRAL 1 .00001 .05 2");assert(cubli_bal.neutral_limit_deg==5);
 assert(!LQR_SeedNeutralReference(saved)); /* active seed is rejected */
 for(i=0;i<3000;i++) {
  memcpy(before,cubli_bal.neutral_rotation,sizeof(before));neutral_tick(200,0,0);
  for(int j=0;j<3;j++) step[j]=cubli_bal.neutral_rotation[j]-before[j];
  assert(norm3(step)<=.5f*DEG_TO_RAD*.005f+1e-7f);
  assert(norm3(cubli_bal.neutral_rotation)<=5*DEG_TO_RAD+1e-7f);
 }
 assert(fabsf(norm3(cubli_bal.neutral_rotation)-5*DEG_TO_RAD)<1e-6f);
 /* Base20deg fall guard still applies even with a5deg control offset. */
 n=norm3(cubli_bal.neutral_rotation);
 for(i=0;i<3;i++) axis[i]=cubli_bal.neutral_rotation[i]/n;
 cross3(base,axis,fallen);
 for(i=0;i<3;i++) fallen[i]=base[i]*cosf(21*DEG_TO_RAD)+fallen[i]*sinf(21*DEG_TO_RAD);
 advance();gravity(fallen[0],fallen[1],fallen[2]);LQR_Balance_Loop();
 assert(cubli_bal.stop_reason==BAL_STOP_ANGLE&&!m1_enabled);
 command("BAL 0");
 command("BNEUTRAL 1 .00025 .5 5 60 60");
 assert(!cubli_bal.neutral_enabled&&norm3(cubli_bal.neutral_rotation)==0);
 command("BNEUTRAL 1 .00025 .5");assert(cubli_bal.neutral_limit_deg==2);
}
static float observer_pivot_rms(float tau) {
 int i,count=0;float error2=0;
 const float initial[3]={-.693037404f,.417914491f,.587406703f};
 const float sensor_y=.035f,sensor_z=.070f,frequency=3.0f,amplitude=.15f;
 setup();assert(LQR_SetAttitudeTau(tau));
 cubli_bal.attitude.is_calibrated=true;gravity(initial[0],initial[1],initial[2]);
 /* Known fixed-pivot motion: IMU specific force includes alpha x r and
    omega x (omega x r), even though its norm remains near1g. */
 for(i=1;i<=1600;i++) {
  float t=i*.005f,f=2*_PI*frequency,omega=amplitude*sinf(f*t),alpha=amplitude*f*cosf(f*t);
  float angle=amplitude/f*(1-cosf(f*t)),c=cosf(angle),s=sinf(angle),truth[3],a[3],w[3]={0},e[3],error;
  truth[0]=initial[0];truth[1]=initial[1]*c+initial[2]*s;truth[2]=initial[2]*c-initial[1]*s;
  a[0]=truth[0];a[1]=truth[1]+(-alpha*sensor_z-omega*omega*sensor_y)/9.80665f;
  a[2]=truth[2]+(alpha*sensor_y-omega*omega*sensor_z)/9.80665f;w[0]=omega*RAD_TO_DEG;
  advance();LQR_Attitude_Update(a,w,.005f);assert(cubli_bal.attitude.valid);
  if(i>400) {cross3(cubli_bal.attitude.gravity,truth,e);error=atan2f(norm3(e),dot3(cubli_bal.attitude.gravity,truth));error2+=error*error;count++;}
 }
 return sqrtf(error2/count)*RAD_TO_DEG;
}
static void test_observer_tuning(void) {
 int i;float fast,slow,a[3]={0,sinf(3*DEG_TO_RAD),cosf(3*DEG_TO_RAD)},w[3]={0},e[3];
 setup();assert(cubli_bal.attitude_tau_s==.25f);
 command("BATTAU .1");command("BATTAU 3");assert(cubli_bal.attitude_tau_s==.25f);
 assert(!LQR_SetAttitudeTau(NAN)&&!LQR_SetAttitudeTau(INFINITY));
 command("BATTAU 1");assert(cubli_bal.attitude_tau_s==1);
 command("EN1 1");assert(m1_enabled);command("BATTAU .5");assert(cubli_bal.attitude_tau_s==1);command("STOP");
 cubli_bal.state=BAL_STATE_WAIT_POSE;assert(!LQR_SetAttitudeTau(.5f));
 cubli_bal.state=BAL_STATE_DISABLED;command("BMODE 4");assert(cubli_bal.attitude_tau_s==.25f);
 assert(LQR_SetAttitudeTau(1));command("CAL_IMU");assert(cubli_bal.attitude_tau_s==.25f);
 fast=observer_pivot_rms(.25f);slow=observer_pivot_rms(1);
 printf("Observer fixed-pivot simulation RMS: .25s=%.4fdeg 1s=%.4fdeg\n",fast,slow);
 assert(slow<.5f*fast); /* Known truth, not a claim about the real cube. */
 setup();assert(LQR_SetAttitudeTau(1));cubli_bal.attitude.is_calibrated=true;gravity(0,0,1);
 for(i=0;i<2000;i++) {advance();LQR_Attitude_Update(a,w,.005f);}
 cross3(cubli_bal.attitude.gravity,a,e);assert(atan2f(norm3(e),dot3(cubli_bal.attitude.gravity,a))*RAD_TO_DEG<.002f);
 a[0]=NAN;LQR_Attitude_Update(a,w,.005f);assert(!cubli_bal.attitude.valid);
}
int main(void) {
 int i;float u;uint32_t saved;
 setvbuf(stdout,NULL,_IONBF,0);
 test_auto_edge();
 test_corner_countdown();
 test_corner_auto();
 test_corner_trim();
 test_corner_verified_control_law();
 test_corner_immediate_feedback();
 test_corner_tight_entry();
 test_neutral_search();
 test_neutral_five_degree_seed();
 test_observer_tuning();
 setup();command("DBG 2");command("EN1 1");command("EN2 1");command("EN3 1");assert(m1_enabled&&m2_enabled&&m3_enabled);
 command("M1 20");command("M2 -30");command("M3 40");assert(target_m1==20&&target_m2==-30&&target_m3==40);
 command("M 180 -180 0");assert(target_m1==180&&target_m2==-180&&target_m3==0);
 command("M2 nan");command("M2 8junk");assert(target_m2==-180);
 command("ACC 100");assert(velocity_ramp_accel==400);command("EN1 0");command("EN2 0");command("EN3 0");
 command("ACC 1000");assert(velocity_ramp_accel==1000);command("ACC 1501");assert(velocity_ramp_accel==1000);command("ACC 400");
 command("L 330");assert(velocity_limit==330);command("L 999");assert(velocity_limit==360);command("L 180");
 reset_motion_state(1);shaft_velocity_sp[1]=10;
 for(i=0;i<50;i++){fake_us+=1000;fake_ms++;ramp_velocity_target(-10,1);}
 assert(fabsf(shaft_velocity_sp[1]+10)<.001f);
 command("APP 1");assert(!m1_enabled&&!m2_enabled&&!m3_enabled&&target_m1==0&&target_m2==0);
 command("EN2 1");command("M2 20");assert(!m2_enabled&&target_m2==0);
 command("BAL 1");assert(cubli_bal.state==BAL_STATE_DISABLED);
 command("CAL_IMU");for(i=0;i<200;i++){float a[3]={0,0,1},w[3]={0};advance();LQR_Attitude_Update(a,w,.005f);}
 assert(cubli_bal.attitude.is_calibrated&&cubli_bal.state==BAL_STATE_DISABLED&&!m1_enabled);
 command("BAXIS 1 2 3");command("BDIR 1 1 1");command("BMODE 4");command("LQR 28 2.2 .07 .006");assert(LQR_GetGains(0)->K_theta==28);
 gravity(.577350269f,.577350269f,.577350269f);command("BAL 1");assert(cubli_bal.state==BAL_STATE_DISABLED);
 command("BZERO");command("BAL 1");LQR_Motor_Loop();advance();gravity(.577350269f,.577350269f,.577350269f);LQR_Balance_Loop();
 assert(cubli_bal.state==BAL_STATE_STANDBY&&!m1_enabled);
 for(i=0;i<21;i++){advance();gravity(.577350269f,.577350269f,.577350269f);LQR_Motor_Loop();LQR_Balance_Loop();}
 assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled&&m2_enabled&&m3_enabled);
 command("BAL 0");command("BMODE 1");gravity(0,0,1);command("BZERO");gravity(0,sinf(.02f),cosf(.02f));command("BAL 1");
 for(i=0;i<22;i++){advance();gravity(0,sinf(.02f),cosf(.02f));LQR_Motor_Loop();LQR_Balance_Loop();}
 assert(m1_enabled&&!m2_enabled&&!m3_enabled);u=cubli_bal.control_u[0];assert(u>0);
 sensor_direction[0]=-1;advance();gravity(0,sinf(.02f),cosf(.02f));LQR_Balance_Loop();assert(fabsf(cubli_bal.control_u[0]-u)<.001f);
 advance();gravity(0,sinf(.4f),cosf(.4f));LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT&&!m1_enabled);
 advance();gravity(0,0,1);LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT);
 /* Permute M1 onto the IMU Y axis: tilt about X must not drive M1. */
 command("BAL 0");command("BAXIS 2 3 1");command("BDIR 1 1 1");gravity(0,0,1);command("BZERO");command("BAL 1");
 for(i=0;i<22;i++){advance();gravity(sinf(-.02f),0,cosf(.02f));LQR_Motor_Loop();LQR_Balance_Loop();}
 assert(m1_enabled && cubli_bal.control_u[0]>0.5f);
 command("BAXIS 1 1 3");assert(cubli_bal.body_axis[0]==1);
 /* A single verified edge must not silently confirm the other wheel axes. */
 setup();command("BAXIS1 2");command("BDIR1 1");
 assert(cubli_bal.axes_confirmed_mask==1&&cubli_bal.directions_confirmed_mask==1);
 command("BCAP 11");assert(fabsf(cubli_bal.capture_angle-10*DEG_TO_RAD)<.0001f);
 command("BCAP 8");assert(fabsf(cubli_bal.capture_angle-8*DEG_TO_RAD)<.0001f);command("BCAP 10");
 command("BTRIP 61");assert(fabsf(cubli_bal.max_angle_trip-20*DEG_TO_RAD)<.0001f);
 command("BTRIP 60");assert(fabsf(cubli_bal.max_angle_trip-60*DEG_TO_RAD)<.0001f);
 command("BTRIP 18");assert(fabsf(cubli_bal.max_angle_trip-18*DEG_TO_RAD)<.0001f);command("BTRIP 20");
 assert(LQR_AxesConfirmed()&&LQR_DirectionsConfirmed());
 command("BMODE 4");assert(!LQR_AxesConfirmed()&&!LQR_DirectionsConfirmed());command("BMODE 1");
 command("APP 1");cubli_bal.attitude.is_calibrated=true;gravity(0,0,1);
 command("BREF 0 0 0");assert(!cubli_bal.reference_valid[0]);command("BREF 0 1 0");assert(!cubli_bal.reference_valid[0]);
 set_motor_enable(1,1);command("BREF 0 0 1");assert(!cubli_bal.reference_valid[0]);set_motor_enable(1,0);
 command("BSTART 3");assert(cubli_bal.state==BAL_STATE_DISABLED); /* Target must be captured deliberately. */
 command("BZERO");command("BREF 0 0 1");assert(cubli_bal.reference[0][2]==1);command("BSTART 3");assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled);
 for(i=0;i<21;i++){advance();gravity(0,0,1);LQR_Balance_Loop();assert(!m1_enabled);}
 assert(cubli_bal.state==BAL_STATE_COUNTDOWN);
 saved=cubli_bal.countdown_deadline_ms;command("BCAP 2");assert(cubli_bal.capture_angle==10*DEG_TO_RAD);
 cubli_bal.attitude.gyro_rad[1]=.4f;advance();gravity(0,0,1);LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_COUNTDOWN&&cubli_bal.countdown_deadline_ms==saved);cubli_bal.attitude.gyro_rad[1]=0;
 advance();gravity(-sinf(.2f),0,cosf(.2f));LQR_Balance_Loop();
 assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled&&!LQR_CountdownRemainingMs());
 assert(cubli_bal.reference[0][0]==0&&cubli_bal.reference[0][2]==1);
 for(i=0;i<21;i++){advance();gravity(0,0,1);LQR_Balance_Loop();assert(!m1_enabled);}
 assert(cubli_bal.state==BAL_STATE_COUNTDOWN);
 for(i=0;i<600;i++){advance();gravity(0,0,1);LQR_Balance_Loop();assert(!m1_enabled);}
 assert(cubli_bal.state==BAL_STATE_STANDBY&&cubli_bal.reference_valid[0]);
 for(i=0;i<22;i++){advance();gravity(0,0,1);LQR_Motor_Loop();LQR_Balance_Loop();}
 assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled&&!m2_enabled&&!m3_enabled);
 advance();gravity(-sinf(.3f),0,cosf(.3f));LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled); /* 17.2deg is inside the requested 20deg limit. */
 /* Falling perpendicular to the controlled edge is still a real fall. */
 advance();gravity(0,sinf(.4f),cosf(.4f));LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT&&!m1_enabled);
 for(i=0;i<30;i++){advance();gravity(0,0,1);LQR_Balance_Loop();}assert(cubli_bal.state==BAL_STATE_FALL_PROTECT);
 command("BSTART 3");assert(cubli_bal.state==BAL_STATE_WAIT_POSE);command("STOP");
 advance();gravity(0,0,1);LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_DISABLED&&!m1_enabled&&safety_trip==1);
 setup();command("APP 1");command("BAXIS1 2");command("BDIR1 1");cubli_bal.attitude.is_calibrated=true;gravity(0,0,1);
 command("BZERO");command("BSTART 3");cubli_bal.attitude.gyro_rad[1]=.2f;
 for(i=0;i<621;i++){advance();gravity(0,0,1);LQR_Balance_Loop();assert(!m1_enabled);}
 assert(cubli_bal.state==BAL_STATE_WAIT_POSE);cubli_bal.attitude.gyro_rad[1]=0;
 command("BSTART 3");fake_ms+=1501;Control_SafetyTick();assert(safety_trip==2&&!m1_enabled);
 /* Fixture threshold remains latched and covers cross-axis falls too. */
 setup();command("APP 1");command("BAXIS1 2");command("BDIR1 1");cubli_bal.attitude.is_calibrated=true;gravity(0,0,1);
 command("BTRIP 60");command("BZERO");command("BAL 1");
 for(i=0;i<22;i++){advance();gravity(0,0,1);LQR_Motor_Loop();LQR_Balance_Loop();}
 assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled);
 command("BTRIP 20");assert(fabsf(cubli_bal.max_angle_trip-60*DEG_TO_RAD)<.0001f); /* Active changes rejected. */
 advance();gravity(-sinf(59*DEG_TO_RAD),0,cosf(59*DEG_TO_RAD));LQR_Balance_Loop();assert(m1_enabled);
 advance();gravity(-sinf(61*DEG_TO_RAD),0,cosf(61*DEG_TO_RAD));LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT&&!m1_enabled);
 advance();gravity(0,0,1);LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT);
 command("BAL 0");command("BAL 1");
 for(i=0;i<22;i++){advance();gravity(0,0,1);LQR_Motor_Loop();LQR_Balance_Loop();}
 advance();gravity(0,sinf(61*DEG_TO_RAD),cosf(61*DEG_TO_RAD));LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT&&!m1_enabled);
 /* Torque identification must time out on the MCU, without a host STOP. */
 setup();command("APP 1");cubli_bal.attitude.is_calibrated=true;gravity(0,0,1);
 command("BPROBE 0.75 80");assert(!m1_enabled);command("BAXIS1 2");
 assert(!cubli_bal.directions_confirmed_mask);command("BPROBE 6.1 80");assert(!m1_enabled);
 command("BPROBE 0.75 80");assert(cubli_bal.state==BAL_STATE_PROBE&&m1_enabled&&!m2_enabled&&!m3_enabled);
 LQR_Motor_Loop();assert(fabsf(voltage[0].q-.75f)<.0001f);
 fake_us+=79000;fake_ms+=79;gravity(0,0,1);LQR_Motor_Loop();assert(m1_enabled);
 fake_us+=1000;fake_ms++;gravity(0,0,1);LQR_Motor_Loop();assert(!m1_enabled&&cubli_bal.state==BAL_STATE_DISABLED&&voltage[0].q==0);
 command("BPROBE -0.75 80");LQR_Motor_Loop();assert(voltage[0].q<0);
 advance();gravity(-sinf(.3f),0,cosf(.3f));LQR_Balance_Loop();assert(!m1_enabled&&cubli_bal.state==BAL_STATE_FALL_PROTECT);
 /* Explicit long fixture impulses retain the MCU timeout and pose/rate guards. */
 command("BAL 0");gravity(0,0,1);command("U 6");
 command("BPROBE 6 121");assert(!m1_enabled);
 command("BPROBE -6 120");LQR_Motor_Loop();assert(voltage[0].q==-6&&m1_enabled&&!m2_enabled&&!m3_enabled&&cubli_bal.probe_wheel_limit==100);
 fake_us+=119000;fake_ms+=119;gravity(0,0,1);LQR_Motor_Loop();assert(m1_enabled);
 fake_us+=1000;fake_ms++;gravity(0,0,1);LQR_Motor_Loop();assert(!m1_enabled&&cubli_bal.state==BAL_STATE_DISABLED&&voltage[0].q==0);
 command("BPROBE 6 80");assert(cubli_bal.probe_wheel_limit==60);shaft_velocity[0]=61;LQR_Motor_Loop();assert(!m1_enabled);
 command("BAL 0");shaft_velocity[0]=0;command("BPROBE 6 120");shaft_velocity[0]=95;LQR_Motor_Loop();assert(m1_enabled);
 shaft_velocity[0]=101;LQR_Motor_Loop();assert(!m1_enabled&&cubli_bal.state==BAL_STATE_FALL_PROTECT);
 command("BAL 0");shaft_velocity[0]=0;command("BPROBE 6 120");cubli_bal.attitude.gyro_rad[1]=2.01f;advance();gravity(0,0,1);LQR_Balance_Loop();assert(!m1_enabled);
 command("BAL 0");cubli_bal.attitude.gyro_rad[1]=0;
 command("U 4");command("BPROBE 6 40");assert(!m1_enabled);
 /* Telemetry must exceed the raised wheel threshold, not clip at 300. */
 setup();lpf_velocity[0].Tf=0;fake_us+=1000;fake_ms++;encoder_raw[0]=1000;updateSensor(0);assert(shaft_velocity[0]>350);
 setup();command("APP 1");command("BAXIS1 2");command("BDIR1 -1");command("L 330");
 cubli_bal.attitude.is_calibrated=true;gravity(0,0,1);command("BZERO");command("BAL 1");
 for(i=0;i<22;i++){advance();gravity(0,0,1);LQR_Balance_Loop();}
 assert(m1_enabled);shaft_velocity[0]=340;advance();gravity(0,0,1);LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT&&!m1_enabled);
 /* A held body with a spinning wheel must not receive the full wheel term
    immediately at handover; it must reach the configured term smoothly. */
 setup();command("APP 1");command("BAXIS1 2");command("BDIR1 -1");command("LQR1 0 0 -0.03 0");
 cubli_bal.attitude.is_calibrated=true;gravity(0,0,1);command("BZERO");command("BAL 1");
 for(i=0;i<22;i++){advance();gravity(0,0,1);LQR_Balance_Loop();}
 assert(cubli_bal.state==BAL_STATE_BALANCING);u=cubli_bal.control_u[0];
 for(i=0;i<400;i++) {
  advance();gravity(0,0,1);shaft_velocity[0]=10;LQR_Balance_Loop();
  assert(fabsf(cubli_bal.control_u[0]-u)<.0021f);u=cubli_bal.control_u[0];
  if(fake_ms-cubli_bal.balance_since_ms<500) assert(fabsf(u)<.0001f);
 }
 assert(fabsf(u-.3f)<.0001f);
 setup();command("EN1 1");fake_ms+=1501;main_alive_ms=fake_ms;Control_SafetyTick();assert(safety_trip==2&&!m1_enabled);
 setup();command("EN1 1");mock_gpio.IDR&=~GPIO_PIN_9;Control_SafetyTick();assert(safety_trip==3&&!m1_enabled);
 setup();command("EN1 1");fake_us+=6000;fake_ms+=6;Control_Heartbeat();Control_SafetyTick();assert(safety_trip==8&&!m1_enabled);
 /* Arm a second axis after its idle sample has aged: acquire feedback first. */
 setup();command("EN1 1");fake_us+=80000;fake_ms+=80;Control_Heartbeat();main_alive_ms=fake_ms;updateSensor(0);
 saved=encoder_diag[2].samples;command("EN3 1");Control_SafetyTick();
 assert(m1_enabled&&m3_enabled&&!safety_trip&&encoder_diag[2].last_us==fake_us&&encoder_diag[2].samples>saved);
 /* Failed pre-arm feedback must leave every power stage disabled. */
 setup();command("EN1 1");encoder_ok=0;command("EN3 1");assert(safety_trip==6&&!m1_enabled&&!m3_enabled);
 setup();command("EN1 1");saved=encoder_diag[0].last_us;command("FREQ 1");assert(encoder_diag[0].last_us==saved);Control_SafetyTick();assert(!safety_trip);
 setup();command("EN1 1");encoder_ok=0;move(3,0);assert(safety_trip==6);
 setup();controller=Type_velocity_openloop;command("M1 20");assert(target_m1==0);motor_aligned[1]=0;command("EN2 1");assert(!m2_enabled);
 setup();controller=Type_velocity_openloop;voltage[0].q=5;shaft_velocity[0]=0;move(3,0);fake_us+=805000;fake_ms+=805;move(3,0);assert(safety_trip==7);
 /* Legitimate rotating saturation must NOT latch the previous 1.8V derating. */
 setup();controller=Type_velocity;shaft_velocity_sp[0]=180;
 for(i=0;i<1100;i++) {fake_us+=1000;fake_ms++;encoder_raw[0]=(encoder_raw[0]+26)%16384;move(180,0);}
 assert(!safety_trip && fabsf(voltage[0].q)>4.5f);
 setup();velocityOpenloop(0,0);assert(TIM1->CCR1==TIM1->CCR2&&TIM1->CCR2==TIM1->CCR3);
 /* Initial motion must settle before it can become a pole-pair endpoint. */
 setup();diagnostic_busy=1;settle_drift_reads=55;saved=fake_ms;
 assert(align_wait_still(0,&u)&&fake_ms-saved>=240);
 settle_drift_reads=10000;saved=fake_ms;assert(!align_wait_still(0,&u)&&fake_ms-saved==1600);
 encoder_ok=0;assert(!align_wait_still(0,&u));
 setup();diagnostic_busy=1;model_on=1;memset(motor_mapping_valid,0,3);memset(motor_aligned,0,3);
 cubli_bal.axes_confirmed_mask=cubli_bal.directions_confirmed_mask=7;
 memset(cubli_bal.reference_valid,1,sizeof(cubli_bal.reference_valid));
 assert(Motor_alignSensor(0)==0 && motor_sensor_map[0]==2 && sensor_direction[0]==1);
 assert(cubli_bal.directions_confirmed_mask==6&&!cubli_bal.reference_valid[0]&&!cubli_bal.reference_valid[3]&&cubli_bal.reference_valid[1]&&cubli_bal.reference_valid[2]);
 assert(Motor_alignSensor(1)==0 && motor_sensor_map[1]==0 && sensor_direction[1]==-1);
 assert(cubli_bal.directions_confirmed_mask==4&&!cubli_bal.reference_valid[1]&&cubli_bal.reference_valid[2]);
 assert(Motor_alignSensor(2)==0 && motor_sensor_map[2]==1 && sensor_direction[2]==1);
 assert(!cubli_bal.directions_confirmed_mask&&!cubli_bal.reference_valid[2]&&cubli_bal.axes_confirmed_mask==7);
 assert(!m1_enabled&&!m2_enabled&&!m3_enabled);
 /* A different M2 pole count must fail alignment rather than silently pass. */
 setup();diagnostic_busy=1;model_on=1;memset(motor_mapping_valid,0,3);memset(motor_aligned,0,3);true_pp[1]=14;
 assert(Motor_alignSensor(1)!=0&&!motor_aligned[1]&&!m2_enabled);
 diagnostic_busy=0;command("PP2 14");assert(motor_pole_pairs[1]==14&&!motor_aligned[1]);
 diagnostic_busy=1;assert(Motor_alignSensor(1)==0&&motor_aligned[1]);true_pp[1]=7;
 setup();diagnostic_busy=1;model_on=0;cubli_bal.directions_confirmed_mask=7;
 memset(cubli_bal.reference_valid,1,sizeof(cubli_bal.reference_valid));
 assert(Motor_alignSensor(1)!=0&&!motor_aligned[1]&&!m2_enabled);
 assert(cubli_bal.directions_confirmed_mask==5&&!cubli_bal.reference_valid[1]&&!cubli_bal.reference_valid[3]&&cubli_bal.reference_valid[0]&&cubli_bal.reference_valid[2]);
 puts("PASS: three-axis control, 50ms ramp, saturation, encoder freshness, balance latch, signed mapping and per-axis pole-pair sweep");
 return 0;
}
