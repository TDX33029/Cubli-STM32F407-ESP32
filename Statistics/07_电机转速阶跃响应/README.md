# 图 7：三电机多级转速阶跃响应

> 本实验与 `08_IMU校准` 是最早完成的两项，为保持 01–06 六张分析图的编号不变，
> 统一补号为 07/08，排在末尾。文件夹内脚本均使用相对路径，可直接运行。

## 实验结果

对 Cubli-F407ZGT6 的三个反作用飞轮电机分别施加多级转速阶跃：

```
0% -> 30% -> 60% -> 100% -> 60% -> 30% -> 0%
```

满速按固件 `velocity_limit = 180 rad/s`（约 1719 rpm）计算，各级台阶为
0 / 54 / 108 / 180 / 108 / 54 / 0 rad/s，每级保持 3.5~4 s。

| 轴 | 100% 台阶峰值 | 超调 | 各级稳态最大偏差 |
|----|--------------|------|------------------|
| M1 | 183.5 rad/s | 2.0% | 0.4% |
| M2 | 183.5 rad/s | 2.0% | 0.6% |
| M3 | 181.0 rad/s | 0.5% | 0.3% |

三轴阶跃响应基本一致，稳态误差均 < 1%，100% 台阶在约 1.1 s 内到达。

## 文件

| 文件 | 说明 |
|------|------|
| `m1_stairs.csv` / `m2_stairs.csv` / `m3_stairs.csv` | 原始遥测数据 |
| `plot_stairs_response.m` | MATLAB 静态绘图脚本（三联子图 + 叠加对比） |
| `staircase_response_3motors.png` | 三轴各子图（目标 vs 实测） |
| `staircase_response_compare.png` | 三轴实测叠加对比 |
| `plot_stairs_animated.py` | Python 动画脚本（逐条曲线绘制） |
| `staircase_response_animated.gif` | 动画成品（25 fps） |
| `staircase_response_animated.png` | 动画末帧静态图（全部曲线） |
| `commands/run_m1_stairs.txt` 等 | 三个轴的串口命令序列 |
| `../tools/scope_client.py` | 串口采集客户端（PING 保活 + CSV 记录） |

CSV 每行格式：`t_local_ms, t_rel_ms, raw`，`raw` 列为完整的 `$TELE`/`$STATE`
行；其中 `M#` 为目标转速、`S#` 为实测滤波转速，单位 rad/s。遥测标称 40 ms
一帧（25 Hz），受串口缓冲影响实测平均 **50 ms（20 Hz）**，每段 25.6 s 约 510 帧。

## 复现

1. 串口终端连接 COM10 @ 115200（固件 `DEBUG_LINK_USART=2`，USART2）。
2. 上电后先对齐：`TEST 1`、`TEST 2`、`TEST 3`（每个约 6 s，期间保持 `PING`）。
3. 采集：`py ../tools/scope_client.py COM10 commands/run_m1_stairs.txt m1_stairs.csv`
   （命令序列中 `DBG n` 清除安全锁存，`ENn 1` 使能，随后按台阶下发 `Mn`，
   客户端全程每 200 ms 发 `PING` 保活；M2、M3 同理）。
4. 生成图：
   - 静态图：`matlab -batch "cd('<本目录>'); plot_stairs_response"`
   - 动画：`py plot_stairs_animated.py --save-only`（无窗口，直接写 GIF/PNG）

> 注意：固件在电机武装状态下要求 1.5 s 内有 `PING`，否则安全停机；
> 每条命令后主机会自动输出一帧 `$STATE`。测量中若设备复位（Aligned 清零），
> 需重新执行 `TEST n` 对齐。
