# 图 8：IMU 校准与未校准对比（两段复位后 0–30 s 叠加）

> 本实验与 `07_电机转速阶跃响应` 是最早完成的两项，为保持 01–06 六张分析图的
> 编号不变，统一补号为 07/08，排在末尾。文件夹内脚本均使用相对路径。

## 实验流程

两段数据相互独立，均以 ST-Link 复位为起点：

| 段 | 操作 | 采集 |
|----|------|------|
| A（未校准） | ST-Link 复位，上电后 `Cal=0`、`gyro_bias=0` | 从首帧 `$TELE` 起 30 s |
| B（校准后） | ST-Link 复位 → `CAL_IMU`（200 × 5 ms 静止采样）→ `OK CAL_IMU:DONE` | 从校准完成起 30 s |

遥测标称 40 ms 一帧（25 Hz），受串口缓冲影响实测平均 **50 ms（20 Hz）**，
30 s 约 600 帧；主机每 200 ms 发送 `PING` 保活。两段时间轴均以各自采集起点
为 0，Yaw 相对各自首帧归零后叠加。

## 固件改动

原 `$TELE` 不含积分偏航角、零偏估计与校准标志，因此在
`Core/Src/main.c` 的 `Control_Telemetry()` 中增加：

| 字段 | 含义 | 单位 |
|------|------|------|
| `Yaw` | 陀螺 z 轴积分偏航角 | deg |
| `Bx/By/Bz` | `CAL_IMU` 估计的陀螺零偏 | °/s |
| `Cal` | IMU 校准完成标志 | 0/1 |

固件已重新编译（0 错误 0 警告）并经 ST-Link（SN 190A071F7116303030303032）
烧录验证。

## 结果

| 指标 | 未校准 (Cal=0) | 校准后 (Cal=1) |
|------|----------------|----------------|
| 30 s Yaw 漂移量 | 9.0° | 0.2° |
| 漂移率 | 0.30 °/s (18.1 °/min) | 0.005 °/s (0.3 °/min) |
| 漂移抑制 | — | 55 倍 |

**陀螺零偏估计 (°/s)**

| 轴 | 静置原始均值 | CAL_IMU 估计 |
|----|-------------|--------------|
| X | +0.480 | +0.470 |
| Y | −0.445 | −0.430 |
| Z | +0.315 | +0.300 |

Yaw 为纯陀螺积分，零偏未补偿时按 `gyro_bias_z ≈ 0.30 °/s` 线性漂移；
校准后残差约 0.005 °/s。Pitch/Roll 由互补滤波结合加速度计修正，静态漂移
始终小于 0.05°，因此校准前后差异主要体现在 Yaw 上。

## 文件

| 文件 | 说明 |
|------|------|
| `uncal_30s.csv` | 段 A 原始遥测（复位后未校准 0–30 s） |
| `cal_30s.csv` | 段 B 原始遥测（复位 + CAL_IMU 后 0–30 s） |
| `plot_imu_calibration_overlay.m` | MATLAB 静态叠加绘图脚本 |
| `imu_calibration_overlay.png` | 叠加对比图 |
| `plot_imu_calibration_animated.m` | MATLAB 动画脚本（逐条曲线绘制 + 漂移拟合） |
| `imu_calibration_animated.gif` | 动画成品（20 fps） |
| `imu_calibration_animated.png` | 动画末帧静态图（全部元素） |
| `imu_capture.py` | 自动复位 + 校准 + 采集脚本 |

## 复现

```bash
# 段 A: 复位后未校准采集 30 s
py imu_capture.py COM10 uncal_30s.csv --seconds=30

# 段 B: 复位 + CAL_IMU + 采集 30 s
py imu_capture.py COM10 cal_30s.csv --calibrate --seconds=30

# 静态图
matlab -batch "cd('<本目录>'); plot_imu_calibration_overlay"

# 动画（写 imu_calibration_animated.gif / .png）
matlab -batch "cd('<本目录>'); plot_imu_calibration_animated"
```

注意：`CAL_IMU` 要求 IMU 在线且设备静止（|加速度模长−1|<0.15 g、
|角速度|<3 °/s），否则采样自动重启。
