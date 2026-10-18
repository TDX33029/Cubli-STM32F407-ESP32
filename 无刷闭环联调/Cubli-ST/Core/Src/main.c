/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "MyProject.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float target = 0.0f;       // SimpleFOC global target (sets all motors)
float target_m1 = 0.0f;    // M1 individual target
float target_m2 = 0.0f;    // M2 individual target
float target_m3 = 0.0f;    // M3 individual target
volatile uint8_t m1_enabled = 0;    // M1 驱动使能标志
volatile uint8_t m2_enabled = 0;    // M2 驱动使能标志
volatile uint8_t m3_enabled = 0;    // M3 驱动使能标志
uint8_t telemetry_enabled = 1; // 周期遥测上报开关 (默认开启)
uint32_t tele_counter = 0; // 遥测计时分频计数器

/* 陀螺仪与加速度计数据缓存 */
float imu_accel[3] = {0.0f, 0.0f, 0.0f}; // 单位: g
float imu_gyro[3]  = {0.0f, 0.0f, 0.0f}; // 单位: °/s (dps)
uint8_t imu_online = 0;                  // 传感器在线状态 (1: 在线, 0: 离线)
uint8_t imu_active_type = ACTIVE_IMU;    // 1: LSM6DSRTR, 2: MPU6050

/* 三轴霍尔/磁编码角度读数 (MT6701-1, MT6701-2, MT6701-3) 为闭环做准备 */
int16_t hall_raw[3] = {0, 0, 0};         // 原始 14-bit 数值 (0 ~ 16383)
float hall_angle_deg[3] = {0.0f, 0.0f, 0.0f}; // 角度 (0.00 ~ 359.99°)
uint8_t hall_online[3] = {0, 0, 0};      // 传感器在线标志 (1: 正常, 0: 离线)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void commander_run(void);
void set_motor_enable(uint8_t motor, uint8_t en);
void Boot_PG4_Blink_5s(void);
void Motor_Encoder_OpenLoop_SelfTest(void);
void Control_Stop(void);
void Control_Status(void);
static void Control_Telemetry(void);
static void Control_FastTelemetry(void);
uint8_t Control_Aborted(void);
static uint8_t app_mode=0, debug_motor=1, diagnostic_busy=0, fast_mask=0;
static volatile uint8_t safety_trip=0;
static volatile uint32_t last_ping_ms=0;
static uint32_t main_alive_ms=0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();   /* 板载 ESP32-WROOM32E 无线上位机链路 (PA9/PA10) */
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	printf("\r\n--- STM32F407 System Clock: %ld Hz ---\r\n", HAL_RCC_GetHCLKFreq());
	printf("--- APB1 Timer Clock: %ld Hz ---\r\n", HAL_RCC_GetPCLK1Freq() * 2);
	printf("--- APB2 Timer Clock: %ld Hz ---\r\n", HAL_RCC_GetPCLK2Freq() * 2);

		/* 硬件安全配置检查(确保PD0/PD1/PD2被正确配置为推挽输出) */
		Motor_GPIO_Safety_Check();

		/* 初始化TIM5 32位全硬件微秒定时器与TIM6 1ms时基 */
		TIM5_Micros_Init();
		TIM6_1ms_Init();           // 1ms定时器中断

		/* 上电快速指示 (PG4亮200ms后熄灭，避免开机长时间阻塞，静候上位机指令) */
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
		delay_ms(200);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);

				/* SimpleFOC参数配置 */
				voltage_power_supply = 12.0f;   // V (DRV8313 12V 硬件供电)
				voltage_limit = 5.0f;           // V (动态相电压上限设为 5.0V，兼顾急速启停力矩与电气安全)
				velocity_limit = 180.0f;        // rad/s (相对原 60 rad/s 上限提高 3 倍)
				controller = Type_velocity; // 默认闭环且硬件失能；完成对齐后才能显式使能
			pole_pairs = 7;                 // 极对数

					SimpleFOC_PID_Init();
                    reset_encoder_diagnostics();           // 初始化速度闭环 PID 与低通滤波器
					LQR_Balance_Init();             // 初始化 Cubli 反作用飞轮 LQR 平衡控制器
					reset_closed_loop_state(-1);    // 初始化闭环同步电角度与控制时间戳
				set_motor_enable(0, 0);         // 上电默认失能三路电机，等待上位机自检指令解锁
			printf("3 motors initialized and IDLE (Safe Lock Mode).\r\n");

	/* 初始化陀螺仪/IMU */
	if (imu_active_type == IMU_TYPE_LSM6DSR)
	{
		printf("Init LSM6DSRTR on SPI2 (PB12=CS, PB13=SCK, PB14=MISO, PB15=MOSI)...\r\n");
		if (LSM6DSR_Init() == 0)
		{
			imu_online = 1;
			printf("LSM6DSRTR ready! WHO_AM_I=0x%02X\r\n", LSM6DSR_Read_ID());
		}
		else
		{
			imu_online = 0;
			printf("LSM6DSRTR init failed or not responding (WHO_AM_I=0x%02X)\r\n", LSM6DSR_Read_ID());
		}
	}
	else if (imu_active_type == IMU_TYPE_MPU6050)
	{
		printf("Init MPU-6050 on soft I2C (PB10=SCL, PB11=SDA)...\r\n");
		if (MPU6050_Init() == 0)
		{
			imu_online = 1;
			printf("MPU-6050 ready! WHO_AM_I=0x%02X\r\n", MPU6050_Read_ID());
		}
		else
		{
			imu_online = 0;
			printf("MPU-6050 init failed (WHO_AM_I=0x%02X)\r\n", MPU6050_Read_ID());
		}
	}

		/* MT6701 磁编码器上电自检
		 * 编码器读取方式由编译期宏 ENC_USE_HW_QUAD 决定 (enc_quad.h):
		 *   0 = 三路 I2C 绝对式读取 (原方法, 默认)
		 *   1 = TIM2/TIM5/TIM8 硬件正交解码 (新方法, ABZ 增量输出, 需先初始化定时器)
		 */
#if ENC_USE_HW_QUAD
		ENC_Quad_Init();
		printf("Encoder mode: HW Quadrature (TIM2=M1 PA5/PB3, TIM5=M2 PA0/PA1, TIM8=M3 PC6/PC7), CPR=%u/%u/%u (x4)\r\n", ENC_Quad_CountsPerRev(0),ENC_Quad_CountsPerRev(1),ENC_Quad_CountsPerRev(2));
#else
		printf("Encoder mode: I2C absolute (I2C1 PB6/PB7, I2C2 PF1/PF0, I2C3 PA8/PC9)\r\n");
#endif
		hall_online[0] = (ENC_GetAngle(0, &hall_raw[0], &hall_angle_deg[0]) == 0);
		hall_online[1] = (ENC_GetAngle(1, &hall_raw[1], &hall_angle_deg[1]) == 0);
		hall_online[2] = (ENC_GetAngle(2, &hall_raw[2], &hall_angle_deg[2]) == 0);
#if ENC_USE_HW_QUAD
        printf("Encoder counters: Ready=%d%d%d (not physical online), relative angles=%.1f/%.1f/%.1f deg\r\n",
               hall_online[0],hall_online[1],hall_online[2],hall_angle_deg[0],hall_angle_deg[1],hall_angle_deg[2]);
#else
		printf("MT6701: M1=%s(%.1f deg), M2=%s(%.1f deg), M3=%s(%.1f deg)\r\n",
			hall_online[0] ? "ONLINE" : "OFFLINE", hall_angle_deg[0],
			hall_online[1] ? "ONLINE" : "OFFLINE", hall_angle_deg[1],
			hall_online[2] ? "ONLINE" : "OFFLINE", hall_angle_deg[2]);
#endif

				/* 启动TIM3/TIM4 PWM输出 (TIM1在MX_TIM1_Init后由Motor_GPIO_Safety_Check使能) */
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

				/* 上电默认保持静止与SysTick微秒时基，不自动执行电机开环自检，等待上位机下发 TEST 检测指令 */
				systick_CountMode();
				printf("System initialized. Motors idle, waiting for UpperComputer Self-Test (TEST)...\r\n");

			target = 0.0f;                  // 上电初始保持静止，等待上位机或终端设定
		target_m1 = target;
		target_m2 = target;
		target_m3 = target;
  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
		static uint32_t last_bal_tick = 0;
        static uint32_t last_fast_tick=0;
        static uint8_t reported_trip=0;
#if !ENC_USE_HW_QUAD
        static uint32_t last_idle_tick=0;
        static uint8_t idle_encoder=0;
#endif
#if ENC_USE_HW_QUAD
        static uint32_t last_encoder_tick=0;
#endif

		while(1)
		{
				/* 5ms (200Hz) 高频平衡与姿态控制任务: IMU数据采集、卡尔曼滤波姿态解算与LQR控制循环 */
				uint32_t cur_tick = HAL_GetTick();
                main_alive_ms=cur_tick;
                /* PG5 countdown, PG6 running, PG7 fall/fault, PG8 ready. */
                {
                    uint32_t pins=GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8,on=0;
                    int m=(int)cubli_bal.mode;
                    uint8_t aligned=m<3?motor_aligned[m]:(motor_aligned[0]&&motor_aligned[1]&&motor_aligned[2]);
                    if(cubli_bal.state==BAL_STATE_COUNTDOWN) {
                        uint32_t remaining=LQR_CountdownRemainingMs();
                        if(remaining>1000U?(cur_tick%1000U<150U):(cur_tick%250U<125U)) on|=GPIO_PIN_5;
                    }
                    if(cubli_bal.state==BAL_STATE_BALANCING || cubli_bal.state==BAL_STATE_PROBE) on|=GPIO_PIN_6;
                    if(cubli_bal.state==BAL_STATE_FALL_PROTECT || safety_trip>1 ||
                       (cubli_bal.auto_enabled && cubli_bal.stop_reason!=BAL_STOP_NONE)) on|=GPIO_PIN_7;
                    if(app_mode && !safety_trip && cubli_bal.state!=BAL_STATE_FALL_PROTECT &&
                       LQR_AxesConfirmed() && LQR_DirectionsConfirmed() &&
                       aligned && cubli_bal.reference_valid[m] && cur_tick-last_ping_ms<=1500U &&
                       cubli_bal.attitude.is_calibrated && cubli_bal.attitude.valid &&
                       cubli_bal.pose_tilt_deg<cubli_bal.capture_angle*RAD_TO_DEG) {
                        float *w=cubli_bal.attitude.gyro_rad;
                        if((cubli_bal.pose_tilt_deg<3 && w[0]*w[0]+w[1]*w[1]+w[2]*w[2]<0.04f) ||
                           (cur_tick%500U)<250U) on|=GPIO_PIN_8;
                    }
                    GPIOG->BSRR=on|((pins&~on)<<16);
                }
                if(safety_trip && reported_trip!=safety_trip) {
                    Control_Stop(); USART_ClearCommands();
                    printf("ERR SAFETY_TRIP:%d\r\n",safety_trip);
                }
                reported_trip=safety_trip;
                if(fast_mask && cur_tick-last_fast_tick>=((app_mode && cubli_bal.state!=BAL_STATE_PROBE)?20U:10U)) {
                    last_fast_tick=cur_tick;
                    if(USART_TxFree()>(app_mode && cubli_bal.mode==BAL_MODE_CORNER?280:160)) Control_FastTelemetry();
                }
#if !ENC_USE_HW_QUAD
                /* Observe one disabled wheel per millisecond even while another
                   axis runs; its coasting angle and enable-time sample stay current. */
                if(cur_tick-last_idle_tick>=1) {
                    uint8_t attempts;
                    last_idle_tick=cur_tick;
                    for(attempts=0;attempts<3;attempts++) {
                        uint8_t axis=idle_encoder;
                        idle_encoder=(idle_encoder+1)%3;
                        if((axis==0 && !m1_enabled) || (axis==1 && !m2_enabled) ||
                           (axis==2 && !m3_enabled)) {updateSensor(axis);break;}
                    }
                }
#endif
#if ENC_USE_HW_QUAD
                /* Keep freewheeling/disabled encoders sampled at >=1kHz, not 25Hz. */
                if(cur_tick-last_encoder_tick>=1) {
                    last_encoder_tick=cur_tick;
                    if(!m1_enabled) updateSensor(0);
                    if(!m2_enabled) updateSensor(1);
                    if(!m3_enabled) updateSensor(2);
                }
#endif
				if((cur_tick - last_bal_tick) >= 5)
				{
					float dt_bal = (float)(cur_tick - last_bal_tick) * 0.001f;
					last_bal_tick = cur_tick;

					/* 读取六轴 IMU 数据 */
					if (imu_online)
					{
						if (imu_active_type == IMU_TYPE_LSM6DSR)
						{
							imu_online=(LSM6DSR_Get_Data(imu_accel, imu_gyro)==0);
						}
						else if (imu_active_type == IMU_TYPE_MPU6050)
						{
							imu_online=(MPU6050_Get_Data(imu_accel, imu_gyro)==0);
						}

						/* 重力向量互补滤波，适用于边沿与顶点姿态 */
						if(imu_online) LQR_Attitude_Update(imu_accel, imu_gyro, dt_bal);
                        else cubli_bal.attitude.valid=false;
					}

					/* 执行 LQR 状态反馈平衡控制计算 (仅在平衡激活或校准时接管电机) */
					LQR_Balance_Loop();
				}

				/* 40ms (25Hz) 周期任务：遥测上报与工作指示 (降低串口阻塞比重，保障FOC高频连续换向) */
				if(time1_cntr >= 40)
				{
					time1_cntr = 0;
					tele_counter++;


					/* 依据动态映射表获取每个电机对应的编码器实时机械角度 (直接读取FOC核心更新的角度，避免总线重复阻塞) */


                if((telemetry_enabled || fast_mask) && (tele_counter%5)==0 && cubli_bal.state!=BAL_STATE_PROBE) Control_Status();
						if(telemetry_enabled && !fast_mask && (tele_counter%5)!=0 && USART_TxFree() > 512)
						{
							/* 扩展标准化遥测格式 (包含三电机目标速度、相电压、使能状态、电角度、六轴陀螺仪加速度、通道映射后MT6701角度H1/H2/H3、实测滤波转速S1/S2/S3、解算Pitch/Roll姿态角及平衡状态Bal) */
                            Control_Telemetry();

						}


				/* 200ms (5次) 翻转LED指示灯: PG0对应M1, PG1对应M2, PG2对应M3 */
				if((tele_counter % 5) == 0)
				{
					if(m1_enabled) HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_0);
					else HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);

					if(m2_enabled) HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);
					else HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);

					if(m3_enabled) HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2);
					else HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
				}

				/* 1000ms (25次) 输出控制台可读格式(仅在关闭TELE时输出) */
				if(tele_counter >= 25)
				{
					tele_counter = 0;
					if(!telemetry_enabled && !m1_enabled && !m2_enabled && !m3_enabled)
					{
						printf("M1:%.3f[%lu %lu %lu]  M2:%.3f[%lu %lu %lu]  M3:%.3f[%lu %lu %lu]  Vq=%.2f\r\n",
							shaft_angle[0],
							TIM1->CCR1, TIM1->CCR2, TIM1->CCR3,
							shaft_angle[1],
							TIM3->CCR1, TIM3->CCR2, TIM3->CCR3,
							shaft_angle[2],
							TIM4->CCR1, TIM4->CCR2, TIM4->CCR3,
							voltage_limit);
					}
				}
			}

			/* 无刷电机驱动输出: 平衡失能状态下执行常规速度控制; 平衡开启时由 LQR_Balance_Loop 接管相电压 */
			if(app_mode == 0 && !safety_trip && cubli_bal.state == BAL_STATE_DISABLED)
			{
				if(m1_enabled) move(target_m1, 0); else { TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; }
				if(m2_enabled) move(target_m2, 1); else { TIM3->CCR1 = 0; TIM3->CCR2 = 0; TIM3->CCR3 = 0; }
				if(m3_enabled) move(target_m3, 2); else { TIM4->CCR1 = 0; TIM4->CCR2 = 0; TIM4->CCR3 = 0; }
			}

            if(app_mode == 1) LQR_Motor_Loop();
		/* 串口命令解析处理 */
		commander_run();
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/******************************************************************************/
/* 控制三路电机 DRV8313 驱动使能状态 (motor: 0=全部, 1=M1, 2=M2, 3=M3) */
/* Safety latch is checked even during alignment/diagnostics by TIM6 ISR. */
void Control_Heartbeat(void) { last_ping_ms=HAL_GetTick(); }
uint8_t Control_Aborted(void) { return safety_trip; }
void Control_RequestStop(uint8_t reason) {
    if(!safety_trip) safety_trip=reason;
    cubli_bal.auto_enabled=false;
    if(reason>1) cubli_bal.stop_reason=BAL_STOP_FAULT;
    GPIOD->BSRR=(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2)<<16;
    TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=0;
    TIM3->CCR1=TIM3->CCR2=TIM3->CCR3=0;
    TIM4->CCR1=TIM4->CCR2=TIM4->CCR3=0;
    m1_enabled=m2_enabled=m3_enabled=0;
}
void Control_SafetyTick(void) {
    uint32_t now=HAL_GetTick();
    uint8_t armed=m1_enabled || m2_enabled || m3_enabled;
    if(safety_trip) {Control_RequestStop(safety_trip);return;}
    if(armed || cubli_bal.state==BAL_STATE_STANDBY || cubli_bal.state==BAL_STATE_COUNTDOWN || cubli_bal.state==BAL_STATE_WAIT_POSE) {
        if(now-last_ping_ms>1500) {Control_RequestStop(2);return;}
        if(!diagnostic_busy) {
            uint32_t us=micros();
            if((m1_enabled && us-encoder_diag[0].last_us>5000U) ||
               (m2_enabled && us-encoder_diag[1].last_us>5000U) ||
               (m3_enabled && us-encoder_diag[2].last_us>5000U)) {Control_RequestStop(8);return;}
        }
        if(!diagnostic_busy && now-main_alive_ms>30) {Control_RequestStop(5);return;}
        if((m1_enabled && !(GPIOG->IDR&GPIO_PIN_9)) ||
           (m2_enabled && !(GPIOG->IDR&GPIO_PIN_10)) ||
           (m3_enabled && !(GPIOG->IDR&GPIO_PIN_11))) Control_RequestStop(3);
    }
}
void set_motor_enable(uint8_t motor,uint8_t en) {
    int i; uint32_t irq=__get_PRIMASK();
    volatile uint8_t *flags[3]={&m1_enabled,&m2_enabled,&m3_enabled};
    TIM_TypeDef *timers[3]={TIM1,TIM3,TIM4};
    /* Read real feedback before arming, outside the interrupt-masked section.
       Do not fabricate freshness or weaken the running 5ms safety deadline. */
    if(en && !safety_trip) for(i=0;i<3;i++) if((motor==0 || motor==i+1) && !*flags[i]) {
        updateSensor(i);
        if(!sensor_valid[i]) {Control_RequestStop(6);return;}
    }
    __disable_irq();
    for(i=0;i<3;i++) if(motor==0 || motor==i+1) {
        if(en && (safety_trip || HAL_GetTick()-last_ping_ms>1500 ||
                  !(GPIOG->IDR&(GPIO_PIN_9<<i)))) {
            Control_RequestStop(3); break;
        }
        if(en && !*flags[i]) {reset_closed_loop_state(i);reset_motion_state(i);}
        *flags[i]=en?1:0;
        if(en) GPIOD->BSRR=GPIO_PIN_0<<i;
        else {
            GPIOD->BSRR=(GPIO_PIN_0<<i)<<16;
            timers[i]->CCR1=timers[i]->CCR2=timers[i]->CCR3=0;
            voltage[i].q=voltage[i].d=0;
            shaft_velocity_sp[i]=0;
        }
    }
    __set_PRIMASK(irq);
}
void Control_Stop(void) {
    LQR_Balance_Enable(false); set_motor_enable(0,0);
    target=target_m1=target_m2=target_m3=0;
    reset_closed_loop_state(-1);reset_motion_state(-1);
}
static uint8_t control_reset_latch(void) {
    if(HAL_GetTick()-last_ping_ms>1500) {printf("ERR HEARTBEAT (send PING)\r\n");return 0;}
    Control_Stop(); safety_trip=0; main_alive_ms=HAL_GetTick(); return 1;
}
static void report_int(const char *key,int32_t value) {UART_WriteText(",");UART_WriteText(key);UART_WriteText(":");UART_WriteInt(value);}
static void report_float(const char *key,float value,uint8_t decimals) {UART_WriteText(",");UART_WriteText(key);UART_WriteText(":");UART_WriteFixed(value,decimals);}
static void report_en(void) {UART_WriteText(",EN:");UART_WriteInt(m1_enabled);UART_WriteInt(m2_enabled);UART_WriteInt(m3_enabled);}
static void Control_FastTelemetry(void) {
    int i;char key[3]={'R','1',0};
    UART_WriteText("$FAST,T:");UART_WriteUInt(micros());
    for(i=0;i<3;i++) if(fast_mask&(1U<<i)) {
        key[1]=(char)('1'+i);key[0]='R';report_float(key,shaft_velocity_sp[i],2);
        key[0]='V';report_float(key,shaft_velocity[i],2);key[0]='U';report_float(key,voltage[i].q,2);
    }
    if(app_mode) {
        report_float("Tilt",cubli_bal.tilt_deg,2);
        report_float("Angle",cubli_bal.signed_tilt_deg,2);
        float *w=cubli_bal.attitude.gyro_rad;
        report_float("Rate",cubli_bal.mode<3?w[cubli_bal.body_axis[cubli_bal.mode]]:sqrtf(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]),3);
        report_int("Bal",cubli_bal.state);
        if(cubli_bal.state==BAL_STATE_COUNTDOWN) report_int("CountMs",LQR_CountdownRemainingMs());
        if(cubli_bal.state==BAL_STATE_PROBE || cubli_bal.mode==BAL_MODE_CORNER) {
            report_float("Wx",cubli_bal.attitude.gyro_rad[0],3);
            report_float("Wy",cubli_bal.attitude.gyro_rad[1],3);
            report_float("Wz",cubli_bal.attitude.gyro_rad[2],3);
        }
        if(cubli_bal.mode==BAL_MODE_CORNER) {
            report_float("Pit",cubli_bal.attitude.pitch_deg,2);
            report_float("Rol",cubli_bal.attitude.roll_deg,2);
            if(cubli_bal.neutral_enabled) {
                const float *ref=LQR_GetControlReference();
                report_float("NPit",atan2f(ref[1],sqrtf(ref[0]*ref[0]+ref[2]*ref[2]))*RAD_TO_DEG,2);
                report_float("NRol",atan2f(-ref[0],ref[2])*RAD_TO_DEG,2);
            } else {
                report_float("T1",cubli_bal.tilt_int[0],3);
                report_float("T2",cubli_bal.tilt_int[1],3);
                report_float("T3",cubli_bal.tilt_int[2],3);
            }
        }
    }
    UART_WriteText("#\r\n");
}
static void Control_Telemetry(void) {
    UART_WriteText("$TELE");
    report_float("M1",target_m1,2);
    report_float("M2",target_m2,2);
    report_float("M3",target_m3,2);
    report_float("Vq",voltage_limit,2);
    report_en();
    report_float("A1",shaft_angle[0],2);
    report_float("A2",shaft_angle[1],2);
    report_float("A3",shaft_angle[2],2);
    report_float("Gx",imu_gyro[0],1);
    report_float("Gy",imu_gyro[1],1);
    report_float("Gz",imu_gyro[2],1);
    report_float("Ax",imu_accel[0],2);
    report_float("Ay",imu_accel[1],2);
    report_float("Az",imu_accel[2],2);
    report_float("H1",angle_prev[0]*RAD_TO_DEG,1);
    report_float("H2",angle_prev[1]*RAD_TO_DEG,1);
    report_float("H3",angle_prev[2]*RAD_TO_DEG,1);
    report_float("S1",shaft_velocity[0],2);
    report_float("S2",shaft_velocity[1],2);
    report_float("S3",shaft_velocity[2],2);
    report_float("Pit",cubli_bal.attitude.pitch_deg,2);
    report_float("Rol",cubli_bal.attitude.roll_deg,2);
    report_int("Bal",cubli_bal.state);UART_WriteText("#\r\n");
}
void Control_Status(void) {
    if(USART_TxFree()<900) return;
    UART_WriteText("$STATE");
    report_int("App",app_mode);
    report_int("Dbg",debug_motor);
    report_int("Mode",controller==Type_velocity?1:0);
    report_int("BMode",cubli_bal.mode+1);
    report_int("Bal",cubli_bal.state);
    report_int("Cal",cubli_bal.attitude.is_calibrated);
    report_int("IMU",imu_online && cubli_bal.attitude.valid);
    UART_WriteText(",Aligned:");
    UART_WriteInt(motor_aligned[0]);UART_WriteInt(motor_aligned[1]);UART_WriteInt(motor_aligned[2]);
    report_en();
    report_int("Fault",(~GPIOG->IDR>>9)&7UL);
    report_int("Trip",safety_trip);
    report_float("Tilt",cubli_bal.tilt_deg,2);
    report_float("PoseTilt",cubli_bal.pose_tilt_deg,2);
    report_float("Angle",cubli_bal.signed_tilt_deg,2);
    report_float("TripDeg",cubli_bal.max_angle_trip*RAD_TO_DEG,1);
    report_float("CapDeg",cubli_bal.capture_angle*RAD_TO_DEG,2);
    report_int("Auto",cubli_bal.auto_enabled);
    report_int("BReason",cubli_bal.stop_reason);
    report_float("FullSpeed",cubli_bal.full_speed_limit,1);
    report_int("FullMs",LQR_FullSpeedElapsedMs());
    report_int("RetryMs",LQR_AutoRetryRemainingMs());
    report_int("EdgeReady",LQR_AutoPoseReady());
    report_float("Wheel",cubli_bal.mode<3?shaft_velocity[cubli_bal.mode]:
                 fmaxf(fmaxf(fabsf(shaft_velocity[0]),fabsf(shaft_velocity[1])),fabsf(shaft_velocity[2])),2);
    report_float("Rate",cubli_bal.mode<3?cubli_bal.attitude.gyro_rad[cubli_bal.body_axis[cubli_bal.mode]]:
                 sqrtf(cubli_bal.attitude.gyro_rad[0]*cubli_bal.attitude.gyro_rad[0]+
                       cubli_bal.attitude.gyro_rad[1]*cubli_bal.attitude.gyro_rad[1]+
                       cubli_bal.attitude.gyro_rad[2]*cubli_bal.attitude.gyro_rad[2]),3);
    report_float("YawD",cubli_bal.K_yaw_damping,2);report_int("YawTune",1);
    report_float("AttTau",cubli_bal.attitude_tau_s,2);report_int("ATune",1);
    report_float("Trim1",cubli_bal.tilt_int[0],3);report_float("Trim2",cubli_bal.tilt_int[1],3);
    report_float("Trim3",cubli_bal.tilt_int[2],3);report_float("BTrim",cubli_bal.K_tilt_trim,1);report_int("TrimTune",1);
    report_int("NTune",3);report_int("NOn",cubli_bal.neutral_enabled);
    if(cubli_bal.mode==BAL_MODE_CORNER && cubli_bal.reference_valid[3]) {
        const float *ref=LQR_GetControlReference(),*r=cubli_bal.neutral_rotation;
        report_float("NPit",atan2f(ref[1],sqrtf(ref[0]*ref[0]+ref[2]*ref[2]))*RAD_TO_DEG,2);
        report_float("NRol",atan2f(-ref[0],ref[2])*RAD_TO_DEG,2);
        report_float("NOff",sqrtf(r[0]*r[0]+r[1]*r[1]+r[2]*r[2])*RAD_TO_DEG,3);
        report_float("NLimit",cubli_bal.neutral_limit_deg,2);report_float("NRate",cubli_bal.neutral_rate_deg_s,2);
        report_float("NGain",cubli_bal.neutral_gain,6);
    }
    report_float("U1",voltage[0].q,2);
    report_float("U2",voltage[1].q,2);
    report_float("U3",voltage[2].q,2);
    report_int("D1",cubli_bal.body_direction[0]);
    report_int("D2",cubli_bal.body_direction[1]);
    report_int("D3",cubli_bal.body_direction[2]);
    UART_WriteText(",Enc:");UART_WriteInt(sensor_valid[0]);UART_WriteInt(sensor_valid[1]);UART_WriteInt(sensor_valid[2]);
    report_float("VLimit",voltage_limit,2);report_float("SLimit",velocity_limit,2);report_int("EncType",ENC_USE_HW_QUAD);
    report_int("C1",ENC_CountsPerRev(0));
    report_int("C2",ENC_CountsPerRev(1));
    report_int("C3",ENC_CountsPerRev(2));
    report_int("Map1",motor_sensor_map[0]+1);
    report_int("Map2",motor_sensor_map[1]+1);
    report_int("Map3",motor_sensor_map[2]+1);
    report_int("PP1",motor_pole_pairs[0]);
    report_int("PP2",motor_pole_pairs[1]);
    report_int("PP3",motor_pole_pairs[2]);
    report_int("Dir1",sensor_direction[0]);
    report_int("Dir2",sensor_direction[1]);
    report_int("Dir3",sensor_direction[2]);
    report_int("Ref",cubli_bal.reference_valid[cubli_bal.mode]);
    report_int("Axis1",cubli_bal.body_axis[0]+1);report_int("Axis2",cubli_bal.body_axis[1]+1);report_int("Axis3",cubli_bal.body_axis[2]+1);
    report_int("AxesOK",LQR_AxesConfirmed());report_int("DirOK",LQR_DirectionsConfirmed());
    report_int("AxisMask",cubli_bal.axes_confirmed_mask);report_int("DirMask",cubli_bal.directions_confirmed_mask);
    report_int("CountMs",LQR_CountdownRemainingMs());report_int("CornerStart",1);UART_WriteText("#\r\n");
}
/* Exact tokens, finite numbers, and argument counts; no prefix/atoi fallthrough. */
void commander_run(void) {
    char line[USART_REC_LEN],*tok,*name; float v[6]; int n=0,i,idx=-1;
    if(!USART_ReadLine(line)) return;
    name=strtok(line," \t"); if(!name) return;
    for(tok=name;*tok;tok++) *tok=(char)toupper((unsigned char)*tok);
    while((tok=strtok(NULL," \t"))!=NULL) {
        char *end; float f=strtof(tok,&end);
        if(n==6 || *end || !isfinite(f)) {printf("ERR ARGUMENT\r\n");return;} v[n++]=f;
    }
    if(!strcmp(name,"STOP") || !strcmp(name,"S")) {
        Control_RequestStop(1); Control_Stop(); USART_ClearCommands(); printf("OK STOP\r\n");
    } else if(!strcmp(name,"STATE?") || !strcmp(name,"ATT") || !strcmp(name,"BAL_STAT")) {
        Control_Status();
    } else if(!strcmp(name,"?") || !strcmp(name,"HELP") || !strcmp(name,"H")) {
        printf("APP 0=DEBUG/1=WORK; DBG 1..3; MODE 0=open/1=closed; TEST [axis]; ALIGN [axis]\r\n"
               "PP1/PP2/PP3 pairs; PP?; FREQ 1=reset, FREQ?=sample diagnostics\r\n"
               "M1/M2/M3 speed independently; EN1/EN2/EN3 0/1; ACC rad/s^2; STOP; PING (every 250ms)\r\n"
               "BMODE 1..3=edge/4=corner; CAL_IMU; BZERO; BREF gx gy gz; BAXIS1/2/3 1..3; BDIR1/2/3 +/-1; BAL 0/1; BSTART 2..15 seconds; BAUTO 0/1 [2..15 seconds]; BFULL speed; BCAP 0.5..10 deg; BTRIP 10..60 deg; BYAW 0..10; BTRIM 0..10; BPROBE volts ms\r\n"
               "LQR1/2/3 [Kp Kd Kw Ki]; ZERO1/2/3 deg (edge); BU volts; U volts; L rad/s; PID1/2/3 [P I D]; TF seconds\r\n"
               "BNEUTRAL 0/1 [gain rate_deg_s [limit_deg [seed_pitch seed_roll]]]: disarmed corner, limit2..5, Ki/BTRIM=0\r\n"
               "BATTAU .25..2 seconds: disarmed accelerometer correction time constant\r\n"
               "FF[1/2/3] [coulomb_V bemf_V_per_rad_s]; STATE?\r\n"); Control_Status();
    } else if(!strcmp(name,"APP") && n==1 && (v[0]==0 || v[0]==1)) {
        Control_Stop(); app_mode=(uint8_t)v[0]; if(!control_reset_latch()) return; printf("OK APP:%d\r\n",app_mode);
    } else if(!strcmp(name,"DBG") && n==1 && v[0]>=1 && v[0]<=3 && v[0]==(int)v[0]) {
        if(app_mode) {printf("ERR DEBUG_ONLY\r\n");return;}
        Control_Stop(); debug_motor=(uint8_t)v[0]; if(!control_reset_latch()) return; printf("OK DBG:%d\r\n",debug_motor);
    } else if(!strcmp(name,"BAL") && n==1 && (v[0]==0 || v[0]==1)) {
        Control_Stop();
        if(v[0]) {
            if(!app_mode || !imu_online || !control_reset_latch()) {printf("ERR BAL PRECONDITION\r\n");return;}
            LQR_Balance_Enable(true);
            if(cubli_bal.state!=BAL_STATE_STANDBY) {printf("ERR BAL (ALIGN, CAL_IMU, BAXIS, BDIR, BZERO required)\r\n");return;}
        }
        printf("OK BAL:%d\r\n",(int)v[0]);
    } else if(!strcmp(name,"BAUTO") && ((n==1 && (v[0]==0 || v[0]==1)) ||
              (n==2 && v[0]==1 && v[1]>=2 && v[1]<=15 && v[1]==(int)v[1]))) {
        Control_Stop();
        if(v[0]) {
            if(!app_mode || !imu_online || !control_reset_latch() || !LQR_StartAuto(n==2?(uint8_t)v[1]:(cubli_bal.mode==BAL_MODE_CORNER?2:8))) {
                printf("ERR BAUTO (ALIGN/CAL/AXIS/DIR/REF for the selected mode and L>BFULL required)\r\n");return;
            }
        }
        printf("OK BAUTO:%d\r\n",(int)v[0]);
    } else if(!strcmp(name,"BFULL") && n==1 && v[0]>=10 && v[0]<velocity_limit) {
        if(cubli_bal.state!=BAL_STATE_DISABLED || m1_enabled || m2_enabled || m3_enabled) {printf("ERR DISARM_FIRST\r\n");return;}
        cubli_bal.full_speed_limit=v[0];printf("OK BFULL\r\n");
    } else if(!strcmp(name,"BSTART") && n==1 && v[0]>=2 && v[0]<=15 && v[0]==(int)v[0]) {
        Control_Stop();
        if(!app_mode || !imu_online || !control_reset_latch() || !LQR_StartCountdown((uint8_t)v[0])) {
            printf("ERR BSTART (ALIGN/CAL_IMU/BAXIS/BDIR/BZERO required)\r\n");return;
        }
        printf("OK BSTART\r\n");
    } else if(!strcmp(name,"BPROBE") && n==2 && fabsf(v[0])<=6 && v[1]>=20 && v[1]<=120 && v[1]==(int)v[1]) {
        if(!app_mode || cubli_bal.state!=BAL_STATE_DISABLED || m1_enabled || m2_enabled || m3_enabled) {printf("ERR DISARM_FIRST_WORK_ONLY\r\n");return;}
        if(!control_reset_latch() || !LQR_StartProbe(v[0],(uint16_t)v[1])) {printf("ERR BPROBE_PRECONDITION\r\n");return;}
        printf("OK BPROBE\r\n");return; /* Keep the short pulse trace free of a large STATE frame. */
    } else if(!strcmp(name,"BCAP") && n==1 && v[0]>=0.5f && v[0]<=10) {
        if(cubli_bal.state!=BAL_STATE_DISABLED || m1_enabled || m2_enabled || m3_enabled) {printf("ERR DISARM_FIRST\r\n");return;}
        cubli_bal.capture_angle=v[0]*DEG_TO_RAD;printf("OK BCAP\r\n");
    } else if(!strcmp(name,"BTRIP") && n==1 && v[0]>=10 && v[0]<=60) {
        if(cubli_bal.state!=BAL_STATE_DISABLED || m1_enabled || m2_enabled || m3_enabled) {printf("ERR DISARM_FIRST\r\n");return;}
        cubli_bal.max_angle_trip=v[0]*DEG_TO_RAD;printf("OK BTRIP\r\n");
    } else if(!strcmp(name,"BYAW") && n==1 && v[0]>=0 && v[0]<=10) {
        if(cubli_bal.state!=BAL_STATE_DISABLED || m1_enabled || m2_enabled || m3_enabled) {printf("ERR DISARM_FIRST\r\n");return;}
        cubli_bal.K_yaw_damping=v[0];printf("OK BYAW\r\n");
    } else if(!strcmp(name,"BATTAU") && n==1) {
        if(m1_enabled || m2_enabled || m3_enabled || !LQR_SetAttitudeTau(v[0])) {printf("ERR BATTAU (disarm, range.25..2s)\r\n");return;}
        printf("OK BATTAU\r\n");
    } else if(!strcmp(name,"BNEUTRAL") && ((n==1 && (v[0]==0 || v[0]==1)) || ((n==3 || n==4 || n==6) && v[0]==1))) {
        float limit=n>=4?v[3]:2.0f;
        float *rotation=cubli_bal.neutral_rotation;
        if((n>=3 && (v[1]<0.000005f || v[1]>0.00025f || v[2]<0.01f || v[2]>0.5f)) ||
           limit<2 || limit>5 || (n==6 && (fabsf(v[4])>89 || fabsf(v[5])>180))) {printf("ERR NEUTRAL_RANGE\r\n");return;}
        if(v[0]!=0 && sqrtf(rotation[0]*rotation[0]+rotation[1]*rotation[1]+rotation[2]*rotation[2])*RAD_TO_DEG>limit+.001f) {
            printf("ERR NEUTRAL_LIMIT_BELOW_OFFSET\r\n");return;
        }
        if(m1_enabled || m2_enabled || m3_enabled || !LQR_SetNeutralLearning(v[0]!=0)) {
            printf("ERR BNEUTRAL (disarm, corner reference/calibration and Ki/BTRIM=0 required)\r\n");return;
        }
        if(v[0]!=0) cubli_bal.neutral_limit_deg=limit;
        if(n>=3) {cubli_bal.neutral_gain=v[1];cubli_bal.neutral_rate_deg_s=v[2];}
        if(n==6) {
            float p=v[4]*DEG_TO_RAD,r=v[5]*DEG_TO_RAD,seed[3];
            seed[0]=-sinf(r)*cosf(p);seed[1]=sinf(p);seed[2]=cosf(r)*cosf(p);
            if(!LQR_SeedNeutralReference(seed)) {LQR_ResetNeutralLearning();printf("ERR NEUTRAL_SEED_OUTSIDE_BOUNDARY\r\n");return;}
        }
        printf("OK BNEUTRAL\r\n");
    } else if(!strcmp(name,"BTRIM") && n==1 && v[0]>=0 && v[0]<=10) {
        if(cubli_bal.state!=BAL_STATE_DISABLED || m1_enabled || m2_enabled || m3_enabled) {printf("ERR DISARM_FIRST\r\n");return;}
        if(cubli_bal.neutral_enabled && v[0]!=0) {printf("ERR NEUTRAL_INTEGRAL_CONFLICT\r\n");return;}
        LQR_SetTiltTrim(v[0]);printf("OK BTRIM\r\n");
    } else if(!strcmp(name,"BREF") && n==3) {
        float norm=sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);int m=(int)cubli_bal.mode;
        if(cubli_bal.state!=BAL_STATE_DISABLED || m1_enabled || m2_enabled || m3_enabled || m>3 || !LQR_AxesConfirmed() || !LQR_DirectionsConfirmed() ||
           !cubli_bal.attitude.is_calibrated || norm<0.9f || norm>1.1f ||
           (m<3 && fabsf(v[cubli_bal.body_axis[m]])/norm>0.25f)) {printf("ERR BREF (normalized saved reference required)\r\n");return;}
        for(i=0;i<3;i++) cubli_bal.reference[m][i]=v[i]/norm;
        cubli_bal.reference_valid[m]=true;
        LQR_ResetNeutralLearning();
        if(m<3) cubli_bal.gains[m].theta_0=0;
        else for(i=0;i<3;i++) cubli_bal.corner_gains[i].theta_0=0;
        printf("OK BREF\r\n");
    } else if(!strcmp(name,"BMODE") && n==1 && v[0]>=1 && v[0]<=4 && v[0]==(int)v[0]) {
        Control_Stop(); LQR_Balance_SetMode((BalanceMode_e)((int)v[0]-1)); printf("OK BMODE:%d\r\n",(int)v[0]);
    } else if(!strcmp(name,"CAL_IMU") && n==0) {
        Control_Stop();
        if(!imu_online) {printf("ERR IMU_OFFLINE\r\n");return;}
        LQR_Balance_StartCalib(); printf("OK CAL_IMU:START\r\n");
    } else if(!strcmp(name,"BZERO") && n==0) {
        Control_Stop(); printf(LQR_CaptureReference()?"OK BZERO\r\n":"ERR BZERO (CAL_IMU/BAXIS/BDIR/IMU/pose)\r\n");
    } else if(strlen(name)==6 && !strncmp(name,"BAXIS",5) && name[5]>='1' && name[5]<='3' && n==1 && v[0]>=1 && v[0]<=3 && v[0]==(int)v[0]) {
        Control_Stop();LQR_ConfirmAxis((uint8_t)(name[5]-'1'),(uint8_t)v[0]-1);
        printf("OK BAXIS (selected axis; BDIR and BZERO required)\r\n");
    } else if(strlen(name)==5 && !strncmp(name,"BDIR",4) && name[4]>='1' && name[4]<='3' && n==1 && (v[0]==1 || v[0]==-1)) {
        Control_Stop();LQR_ConfirmDirection((uint8_t)(name[4]-'1'),(int)v[0]);printf("OK BDIR\r\n");
    } else if(!strcmp(name,"BAXIS") && n==3) {
        for(i=0;i<3;i++) if(v[i]<1 || v[i]>3 || v[i]!=(int)v[i]) {printf("ERR BAXIS (X=1 Y=2 Z=3)\r\n");return;}
        if(v[0]==v[1] || v[0]==v[2] || v[1]==v[2]) {printf("ERR BAXIS (axes must differ)\r\n");return;}
        Control_Stop();for(i=0;i<3;i++) cubli_bal.body_axis[i]=(uint8_t)v[i]-1;
        cubli_bal.axes_confirmed_mask=7;cubli_bal.directions_confirmed_mask=0;
        memset(cubli_bal.reference_valid,0,sizeof(cubli_bal.reference_valid));
        LQR_ResetNeutralLearning();
        printf("OK BAXIS (BDIR and BZERO required)\r\n");
    } else if(!strcmp(name,"BDIR") && n==3) {
        for(i=0;i<3;i++) if(v[i]!=1 && v[i]!=-1) {printf("ERR BDIR\r\n");return;}
        Control_Stop(); LQR_ResetNeutralLearning();for(i=0;i<3;i++) cubli_bal.body_direction[i]=(int)v[i]; cubli_bal.directions_confirmed_mask=7; printf("OK BDIR\r\n");
    } else if(!strcmp(name,"MODE") && n==1 && (v[0]==0 || v[0]==1)) {
        if(app_mode) {printf("ERR DEBUG_ONLY\r\n");return;}
        Control_Stop(); controller=v[0]?Type_velocity:Type_velocity_openloop; printf("OK MODE:%d\r\n",(int)v[0]);
    } else if((!strcmp(name,"TEST") || !strcmp(name,"CHECK") || !strcmp(name,"ALIGN") || !strcmp(name,"CALIB")) && n<=1) {
        int m=n?(int)v[0]:debug_motor; uint8_t ok;
        if(app_mode || m<1 || m>3 || (n && v[0]!=m)) {printf("ERR DEBUG_AXIS\r\n");return;}
        if(!control_reset_latch()) return;
        diagnostic_busy=1; motor_aligned[m-1]=0;
        printf("$TEST_STEP,M%d#\r\n",m);
        ok=Motor_alignSensor(m-1)==0 && motor_aligned[m-1] && !Control_Aborted();
        diagnostic_busy=0; Control_Stop();
        if(!strcmp(name,"TEST") || !strcmp(name,"CHECK")) {
            printf("$TEST_REPORT,M1:%s:%d:%.1f,M2:%s:%d:%.1f,M3:%s:%d:%.1f,ALL:%s#\r\n",
                   m==1?(ok?"PASS":(motor_mapping_valid[0]?"FAIL_ALIGN":"FAIL_MAP")):"SKIP",m==1&&motor_mapping_valid[0]?100:0,m==1?motor_test_travel_deg[0]:0,
                   m==2?(ok?"PASS":(motor_mapping_valid[1]?"FAIL_ALIGN":"FAIL_MAP")):"SKIP",m==2&&motor_mapping_valid[1]?100:0,m==2?motor_test_travel_deg[1]:0,
                   m==3?(ok?"PASS":(motor_mapping_valid[2]?"FAIL_ALIGN":"FAIL_MAP")):"SKIP",m==3&&motor_mapping_valid[2]?100:0,m==3?motor_test_travel_deg[2]:0,ok?"PASS":"FAIL");
        }
        printf(ok?"OK ALIGN\r\n":"ERR ALIGN\r\n");
    } else if((!strncmp(name,"EN",2) || name[0]=='M' || name[0]=='T' || (name[0]>='1' && name[0]<='3')) &&
              (name[0]=='T' ? !strcmp(name,"T") : (name[0]=='E' ? (strlen(name)==2 || (strlen(name)==3 && name[2]>='1' && name[2]<='3')) :
              (strlen(name)==1 || (strlen(name)==2 && name[1]>='1' && name[1]<='3'))))) {
        uint8_t en=name[0]=='E';
        int explicit_axis=en?(strlen(name)==3?name[2]-'0':0):(name[0]>='1' && name[0]<='3'?name[0]-'0':(strlen(name)==2?name[1]-'0':0));
        float val=0;
        if(app_mode || cubli_bal.state!=BAL_STATE_DISABLED || safety_trip) {printf("ERR DEBUG_ONLY_OR_LATCHED (DBG to reset)\r\n");return;}
        if(n==1) val=v[0];
        else if(n!=3 || explicit_axis || en) {printf("ERR ARGUMENT_COUNT\r\n");return;}
        if(en) {
            if(val!=0 && val!=1) {printf("ERR EN_VALUE\r\n");return;}
            int axis=explicit_axis?explicit_axis:debug_motor;
            if(val && !motor_aligned[axis-1]) {printf("ERR ALIGN_REQUIRED M%d\r\n",axis);return;}
            if(!val) {
                if(axis==1) target_m1=0; else if(axis==2) target_m2=0; else target_m3=0;
            }
            set_motor_enable((uint8_t)axis,(uint8_t)val);
            printf("OK DEBUG M%d\r\n",axis);
        } else {
            if(n==3) {
                if(controller==Type_velocity_openloop && (fabsf(v[0])>OPENLOOP_VELOCITY_LIMIT || fabsf(v[1])>OPENLOOP_VELOCITY_LIMIT || fabsf(v[2])>OPENLOOP_VELOCITY_LIMIT)) {printf("ERR OPEN_LOOP_LIMIT (use MODE 1 for fast motion)\r\n");return;}
                for(i=0;i<3;i++) {
                    if(controller==Type_velocity && v[i]!=0 && !motor_aligned[i]) {printf("ERR ALIGN_REQUIRED M%d\r\n",i+1);return;}
                }
                target_m1=_constrain(v[0],-velocity_limit,velocity_limit);
                target_m2=_constrain(v[1],-velocity_limit,velocity_limit);
                target_m3=_constrain(v[2],-velocity_limit,velocity_limit);
                printf("OK DEBUG M1/M2/M3\r\n");
            } else {
                int axis=explicit_axis?explicit_axis:debug_motor;
                if(controller==Type_velocity_openloop && fabsf(val)>OPENLOOP_VELOCITY_LIMIT) {printf("ERR OPEN_LOOP_LIMIT (use MODE 1 for fast motion)\r\n");return;}
                if(controller==Type_velocity && val!=0 && !motor_aligned[axis-1]) {printf("ERR ALIGN_REQUIRED M%d\r\n",axis);return;}
                val=_constrain(val,-velocity_limit,velocity_limit);
                if(axis==1) target_m1=val; else if(axis==2) target_m2=val; else target_m3=val;
                printf("OK DEBUG M%d\r\n",axis);
            }
        }
    } else if((!strcmp(name,"PP?") || !strcmp(name,"PP")) && n==0) {
        printf("$PP,M1:%d,M2:%d,M3:%d#\r\n",motor_pole_pairs[0],motor_pole_pairs[1],motor_pole_pairs[2]);
    } else if(strlen(name)==3 && name[0]=='P' && name[1]=='P' && name[2]>='1' && name[2]<='3' && n==1) {
        int m=name[2]-'1';
        if(v[0]<1 || v[0]>32 || v[0]!=(int)v[0]) {printf("ERR PP_RANGE (integer 1..32)\r\n");return;}
        if(m1_enabled || m2_enabled || m3_enabled || cubli_bal.state!=BAL_STATE_DISABLED) {printf("ERR DISARM_FIRST\r\n");return;}
        motor_pole_pairs[m]=(uint8_t)v[0];motor_aligned[m]=0;reset_motion_state(m);
        printf("OK PP%d:%d (ALIGN required)\r\n",m+1,motor_pole_pairs[m]);
    } else if((!strcmp(name,"FREQ") || !strcmp(name,"FREQ?")) && n<=1) {
        if(n) {reset_encoder_diagnostics();printf("OK FREQ RESET\r\n");}
        else {
            float seconds=(micros()-encoder_diag_epoch)*1e-6f;
            for(i=0;i<3;i++) {
                EncoderDiagnostics_t *d=&encoder_diag[i];
                UART_WriteText("$FREQ");report_int("M",i+1);report_int("Map",motor_sensor_map[i]+1);report_int("PP",motor_pole_pairs[i]);
                report_int("Hz",seconds>0?(int32_t)(d->samples/seconds+0.5f):0);report_int("GapUs",d->max_gap_us);
                report_int("ReadUs",d->max_read_us);report_int("Errors",d->errors);UART_WriteText("#\r\n");
            }
        }
    } else if(!strncmp(name,"PID",3) || !strncmp(name,"LQR",3) || !strncmp(name,"ZERO",4) || !strncmp(name,"TF",2) || !strncmp(name,"FF",2)) {
        uint8_t pid=!strncmp(name,"PID",3),lqr=!strncmp(name,"LQR",3),zero=!strncmp(name,"ZERO",4),ff=!strncmp(name,"FF",2);
        int len=zero?4:((!strncmp(name,"TF",2) || ff)?2:3);
        char *suffix=name+len;
        if(*suffix>='1' && *suffix<='3') {idx=*suffix-'1';suffix++;}
        if(*suffix=='?') suffix++;
        if(*suffix) {printf("ERR COMMAND\r\n");return;}
        if(idx<0) idx=lqr||zero?(cubli_bal.mode<3?(int)cubli_bal.mode:0):-1;
        if(cubli_bal.state!=BAL_STATE_DISABLED || m1_enabled || m2_enabled || m3_enabled) {printf("ERR DISARM_FIRST\r\n");return;}
        if(ff && n && (n!=2 || v[0]<0.0f || v[0]>1.5f || v[1]<0.0f || v[1]>0.15f)) {printf("ERR FF_RANGE (0~1.5V, 0~0.15V/(rad/s))\r\n");return;}
        if(ff && n!=0 && n!=2) {printf("ERR FF_ARGUMENTS (use FF[1/2/3] [coulomb bemf])\r\n");return;}
        for(i=0;i<3;i++) if(idx<0 || idx==i) {
            if(pid) {
                if(n && (n<2 || n>3 || v[0]<0 || v[0]>10 || v[1]<0 || v[1]>50 || (n==3 && (v[2]<0 || v[2]>1)))) {printf("ERR PID_RANGE\r\n");return;}
                if(n) {pid_velocity[i].P=v[0];pid_velocity[i].I=v[1];if(n==3) pid_velocity[i].D=v[2];}
                printf("OK PID%d: P=%.3f, I=%.3f, D=%.4f\r\n",i+1,pid_velocity[i].P,pid_velocity[i].I,pid_velocity[i].D);
            } else if(lqr) {
                LQR_Gains_t *g=LQR_GetGains(i);
                if(n && (n<3 || n>4 || v[0]<0 || v[0]>200 || v[1]<0 || v[1]>50 || v[2]<-5 || v[2]>5 || (n==4 && (v[3]<-5 || v[3]>5)))) {printf("ERR LQR_RANGE\r\n");return;}
                if(n && cubli_bal.neutral_enabled && (n==4?v[3]:g->K_i)!=0) {printf("ERR NEUTRAL_INTEGRAL_CONFLICT\r\n");return;}
                if(n) LQR_SetGains(i,v[0],v[1],v[2],n==4?v[3]:g->K_i);
                printf("OK LQR%d: P=%.3f, D=%.3f, W=%.4f, I=%.4f\r\n",i+1,g->K_theta,g->K_dtheta,g->K_w,g->K_i);
            } else if(zero) {
                if(n!=1 || fabsf(v[0])>45 || cubli_bal.mode==BAL_MODE_CORNER) {printf("ERR ZERO (edge only, use BZERO for corner)\r\n");return;}
                LQR_SetZeroAngle(i,v[0]); printf("OK ZERO%d\r\n",i+1);
            } else if(ff) {
                float coulomb_v, bemf_v_per_rad_s;
                if(n) velocity_feedforward_set(i,v[0],v[1]);
                velocity_feedforward_get(i,&coulomb_v,&bemf_v_per_rad_s);
                printf("OK FF%d: Coulomb=%.3fV, BEMF=%.5fV/(rad/s)\r\n",i+1,coulomb_v,bemf_v_per_rad_s);
            } else {
                if(n!=1 || v[0]<0.001f || v[0]>0.1f) {printf("ERR TF_RANGE\r\n");return;} lpf_velocity[i].Tf=v[0];
            }
        }
    } else if(!strcmp(name,"ACC") && n==1) {
        if(m1_enabled || m2_enabled || m3_enabled) {printf("ERR DISARM_FIRST\r\n");return;}
        if(v[0]<100.0f || v[0]>1500.0f) {printf("ERR ACC_RANGE (100..1500 rad/s^2)\r\n");return;}
        velocity_ramp_accel=v[0]; printf("OK ACC:%.1f rad/s^2\r\n",velocity_ramp_accel);
    } else if((!strcmp(name,"U") || !strcmp(name,"L") || !strcmp(name,"BU")) && n==1) {
        if(v[0]<=0) {printf("ERR LIMIT\r\n");return;}
        if(!strcmp(name,"L")) velocity_limit=_constrain(v[0],1,360);
        else if(!strcmp(name,"BU")) LQR_SetMaxVoltage(_constrain(v[0],0.1f,voltage_limit));
        else voltage_limit=_constrain(v[0],0.1f,voltage_power_supply*0.577350269f);
        printf("OK LIMIT\r\n");
    } else if(!strcmp(name,"FAST") && n==1 && v[0]>=0 && v[0]<=7 && v[0]==(int)v[0]) {
        fast_mask=(uint8_t)v[0];printf("OK FAST:%d\r\n",fast_mask);
    } else if(!strcmp(name,"TELE") && n==1 && (v[0]==0 || v[0]==1)) {
        telemetry_enabled=(uint8_t)v[0]; printf("OK TELE:%d\r\n",telemetry_enabled);
    } else if(!strcmp(name,"ENC?") || !strcmp(name,"ENC") || !strcmp(name,"HALL")) {
        int16_t raw[3]={0}; float deg[3]={0}; uint8_t ready[3];
        if(m1_enabled || m2_enabled || m3_enabled) {printf("ERR DISARM_FIRST\r\n");return;}
        for(i=0;i<3;i++) ready[i]=ENC_GetAngle(i,&raw[i],&deg[i])==0;
        printf("$ENC,Type:%s,C1:%u,C2:%u,C3:%u,R1:%d,R2:%d,R3:%d,H1:%.2f,H2:%.2f,H3:%.2f,Ready:%d%d%d#\r\n",
               ENC_USE_HW_QUAD?"QUAD":"I2C",ENC_CountsPerRev(0),ENC_CountsPerRev(1),ENC_CountsPerRev(2),
               raw[0],raw[1],raw[2],deg[0],deg[1],deg[2],ready[0],ready[1],ready[2]);
    } else if(!strcmp(name,"IMU")) {
        Control_Status();
    } else if(!strcmp(name,"SENSOR") && n==1 && (v[0]==1 || v[0]==2)) {
        Control_Stop(); cubli_bal.attitude.is_calibrated=false; cubli_bal.attitude.valid=false;
        memset(cubli_bal.reference_valid,0,sizeof(cubli_bal.reference_valid));
        imu_active_type=(uint8_t)v[0];
        imu_online=(imu_active_type==1?LSM6DSR_Init():MPU6050_Init())==0;
        printf("OK SENSOR:%d ONLINE:%d\r\n",imu_active_type,imu_online);
    } else {printf("ERR COMMAND_OR_ARGUMENTS\r\n");}
    Control_Status();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  GPIOD->BSRR=(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2)<<16;
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
