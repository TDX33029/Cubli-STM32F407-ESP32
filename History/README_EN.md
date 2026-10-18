# Cubli-F407ZGT6 Dialogs

[🇨🇳 简体中文](README.md) | [🇬🇧 English](README_EN.md)

Cubli STM32F407ZGT6 Version Update Log

[260327] Version 0.7 – Initial material selection completed and confirmed.

[260419] Version 0.8 – Mainboard MT6701 encoder solution verified.

[260424] Version 1.1-alpha – Issues found in the power protection solution BM3451UNDC; critical issues found in the motor driver DRV8313.

[260424] Version 1.2-alpha – Added a large number of debugging indicator LEDs. Each DRV8313 is equipped with three LEDs to indicate Enable / Host Call / Fault status respectively. Each main module is equipped with LED indicators.

[260607] Version 1.2-alpha – The power management BM3451UNDC has issues that cannot be fixed; switched to the Zhongying SH367103x management solution, and the version was iterated to Version 1.3-alpha. Fixed the existing encoder terminal incompatibility issue.

[260923] Power protection solution verified; awaiting verification of the RC battery charging function. Planned to add ESP32-WROOM for wireless debugging. Version iterated to Version 1.4.

[261007] Version 1.4 completed; fixed known minor issues.

[261018] Unprecedented single-point balancing has been achieved; preparing to migrate to wireless serial port.