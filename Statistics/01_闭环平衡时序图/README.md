# 图 1：闭环平衡过程时序图

## 实验

Cubli 单点（corner）闭环平衡，使用项目已验证的 `serial_corner_auto.py` 流程：

| 项目 | 配置 |
|------|------|
| 模式 | `BMODE 4`（单点/corner） |
| 轴映射 | `BAXIS 2 3 1`，`BDIR -1 -1 -1` |
| LQR | `LQR1/2/3 80 5 -0.030 0` |
| 限压 | `U 6`，`BU 6`（V） |
| 速度上限 | `L 330` rad/s |
| 入位/保护 | `BCAP 0.5°`，`BTRIP 20°`，`BFULL 250` |
| 参考 | `BREF -0.702828789 0.397786931 0.589743377`（Pit≈23.44°, Rol≈50.00°） |
| 启动 | `BAUTO 1 2`（自动等待入位，2 s 倒计时自动接管） |

> 2026-10-24 补充：本机固定 `BREF` 有约 1.3° 重心偏置。复现时应在 `BREF` 后、
> `BAUTO` 前发送 `BNEUTRAL 1 0.00025 0.5 5 23.40 48.60`，并以 **PG8 常亮** 入位；
> 否则飞轮会持续加速补偿偏差，约 3 s 饱和跌倒。详见 05/06 的 README。

## 数据

`corner_run1.csv`：BAUTO 接管于采集起点后 71.8 s，平衡持续 **24.3 s**（脚本发 STOP
正常结束，非跌倒）。

- 采样：`$TELE` 标称 40 ms（25 Hz）一帧；串口缓冲不足时跳过，实测平均 50 ms
- 姿态：`Pit/Rol/Yaw`；目标：`RefPit/RefRol`；角速度：`Wx/Wy/Wz`（已扣零偏）；
  轮速：`S1/S2/S3`；控制电压：`U1/U2/U3`

## 结果

| 指标 | 数值 |
|------|------|
| 稳态 RMS 误差 | Pitch 0.32°，Roll 0.18° |
| 稳态峰峰 | Pitch 1.44°，Roll 1.14° |
| 飞轮稳态峰值 | 65 rad/s |
| 平衡时长 | 24.3 s |

系统在接管瞬间已由手扶进入 ±0.5° 入位窗口，接管后无需重新收敛；飞轮从静止
加速到 -40~-65 rad/s 平衡重力矩，轮速缓慢漂移说明存在轻微重心偏置/扰动。

## 文件

| 文件 | 说明 |
|------|------|
| `corner_run1.csv` | 原始遥测 |
| `capture_corner.py` | 自动配置 + BAUTO + 采集脚本 |
| `plot_balance_timeline.m` | MATLAB 绘图脚本（三子图共享时间轴） |
| `balance_timeline.png` | 输出图 |

## 复现

```bash
py capture_corner.py COM10 corner_run1.csv --seconds=60
matlab -batch "cd('<本目录>'); plot_balance_timeline"
```

现场：三轴对齐 + IMU 校准（设备平放稳定）→ 扶成单点姿态 Pit≈23.4°/Rol≈50.0°
→ PG8 常亮（进入 0.5° 窗口）→ PG5 倒计时 2 s → PG6 亮起松手。

> 注：单点平衡使用 `BMODE 4`，不要用 `BMODE 1`（单边 edge，当前固件下未通过
> 参数验证）。另外 `BAXIS` 命令会清除已采集的参考，必须按
> `BAXIS → BDIR → BREF/BZERO → BAUTO` 的顺序配置。
