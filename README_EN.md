# Cubli-F407ZGT6 Dialogs

[🇨🇳 简体中文](README.md) | [🇬🇧 English](README_EN.md)

Cubli STM32F407ZGT6 Changelog

[260327]Version 0.7, initial bill of materials (BOM) selection has been finalized

[260419]Version 0.8, mainboard MT6701 encoder solution verified

[260424]Version 1.1-alpha, power protection scheme BM3451UNDC has issues, motor driver DRV8313 has fatal issues

[260424]Version 1.2-alpha, added numerous debugging indicator LEDs; each DRV8313 is equipped with three LEDs to display Enable / Host Call / Fault status, and each main module is equipped with LED indicators

[260607]Version 1.2-alpha power management BM3451UNDC has unfixable issues, switched to Sinowealth SH367103x management solution, iterated version to Version 1.3-alpha. Fixed encoder connector incompatibility issues

[260923]Power protection scheme verified, pending validation of RC battery charging function, planning to add ESP32-WROOM for wireless debugging, iterated version to Version 1.4

[261007]Version1.4 completed，fixed some previous issues

[261018]Unprecedented single-point balance has been achieved, ready to switch to the wireless serial port