# MDK工程与保留固件

2026-10-16：当前已烧录`CubeMX-corner-immediate`，单点接管时立即使用完整轮速反馈，入位/倒计时/交接均要求三轴有效且轮速低于5rad/s；单边原交接斜坡保留。源码控制及UART测试通过，ARMCC38输入0错误0警告，烧录校验通过；单点实机验证尚待完成。该目录仅保留HEX、AXF/MAP/调用图、构建配置及校验/审阅记录，已删除本次生成的对象中间文件。之前的entry2s及成功回退固件保留。

2026-10-15清理：保留主工程、当前可调试构建和具有实测依据的回退固件。旧的串口/映射/对齐试验、已被替代的单边阶段、未验证的旧QUAD副本、实验性BTRIM版本及重复中间文件已移出当前工作区，历史仍保存在Git中。详细文件数量、原HEX哈希及保留依据见 `cleanup_manifest.json`。

## 入口和构建依赖

- Keil入口：`CubeMX.uvprojx`；保留主工程选项 `CubeMX.uvoptx`。
- `startup_stm32f407xx.s`、`RTE/`、`DebugConfig/` 是启动/工程配置，保留。
- **`CubeMX/CubeMX.lnp` 与 `CubeMX/CubeMX.sct` 是命令行构建依赖，必须保留。** `build_firmware.py` 从前者读取链接输入，链接器使用后者的Flash/RAM布局。
- `.o/.d/.crf/.dep/.lst`、自动生成的 `build_firmware.lnp` 和个人 `uvguix` 布局不入库；编译时会临时生成对象文件。

## 保留版本

| 目录 | 用途与验证范围 | 保留内容 |
| --- | --- | --- |
| `CubeMX-corner-immediate` | 当前板载单点立即完整反馈版本；入位要求低轮速；已烧录校验和重新自检校准，平衡待验证 | HEX、AXF、MAP、调用图、编译配置、烧录校验及review manifest |
| `CubeMX` | 清理时I2C构建，HEX与旧entry2s版本一致。默认Keil工程输出入口，重新构建后反映最新源码 | HEX、AXF、MAP、调用图、编译配置、链接输入及内存布局文件 |
| `CubeMX-quad` | 清理时源码的F4硬件正交计数构建；编译通过，未烧录或进行实物正交验证 | HEX、AXF、MAP、调用图、编译配置 |
| `CubeMX-corner-entry2s` | 前一版单点联调回退，2秒倒计时、0.5°入位、BTRIM默认0；已烧录校验，尚无该条件下8秒成功结论 | HEX、编译配置、烧录校验记录、review manifest |
| `CubeMX-corner-reviewed` | 接手审阅版；有用户确认的5秒全程无干预单点成功记录 | HEX、编译配置、烧录校验记录、review manifest |
| `CubeMX-corner-yaw-settle` | 原始3秒/5秒单点成功版本；包含对齐端点静止检测修复 | HEX、编译配置、烧录记录 |
| `CubeMX-edge-auto` | M1单边自动检测/保护重入的实机验证回退版本 | HEX、编译配置 |
| `CubeMX-edge-impulse120` | 原始单边5秒成功与短力矩方向试验的回退版本 | HEX、编译配置 |
| `CubeMX-motor-diagnostics` | M1/M3快速反转和高转速实测的调速阶段回退版本 | HEX、编译配置 |

HEX只包含程序；轴映射、对齐结果、IMU校准及已验证平衡运行参数需按相应记录重新建立。原始与复现的单点成功profile分别在 `../../diagnostics/2026-10-14/corner_balance_verified5s_profile.json` 和 `../../diagnostics/2026-10-15/review_corner_verified5s_profile.json`。

## 重新构建

在 `Cubli-ST` 目录运行：

```powershell
python build_firmware.py --encoder i2c
python build_firmware.py --encoder quad
```

两种模式分别输出到 `CubeMX/` 和 `CubeMX-quad/`，各自完整重编38个输入，避免复用不同编码器宏的对象文件。本次清理后两条入口均实际构建成功，0错误、0警告。没有修改源码或烧录设备，设备调试任务继续暂停。

## 历史产物恢复

早期文档中的已删除目录是历史路径，不能视为当前可用固件入口。清理前快照为 `6bb714b`；例如在仓库根目录恢复一份旧HEX用于对照：

```powershell
git restore --source=6bb714b -- "无刷闭环联调/Cubli-ST/MDK-ARM/CubeMX-corner-trim/CubeMX.hex"
```

这只恢复指定历史文件，不会重置当前源码。恢复的实验版本应结合其当时的验证记录使用。
