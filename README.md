# Cubli Wireless

[🇨🇳 简体中文](README.md) | [🇬🇧 English](README_EN.md)

> LQR 平衡控制器算法 · STM32F407ZGT6 + ESP32-WROOM32 联合工程 支持二次开发学习

## 简介

本工程为 Cubli 平衡控制器的完整开源实现，包含固件、硬件、机械结构、上位机软件及全部必要文档，所有材料均已提供，不再赘述。

## 编译与构建

| 组件 | 工具链 |
| --- | --- |
| STM32F407ZGT6 固件 | Keil MDK V5.06 |
| ESP32-WROOM32 固件 | PlatformIO |

## 文件结构

| 目录 | 内容 |
| --- | --- |
| `Docs/` | 必要的数据手册文档 |
| `Firmware/` | STM32F407ZGT6 与 ESP32-WROOM32 固件源码 |
| `Hardware/` | PCB 相关资源文件 |
| `History/` | 历史提交归档 |
| `Mechanical/` | 机械结构相关资源文件 |
| `Software/` | 上位机控制器源码与可执行文件 |

## 演示与预览

### 平衡控制器演示

https://github.com/user-attachments/assets/2c3d52b1-5e99-43fa-8d38-f5f9dfa2442f

### PCB 渲染与 Layout

| PCB 正面渲染 | PCB 背面渲染 |
| :---: | :---: |
| <img width="420" alt="MB-3D-TOP" src="https://github.com/user-attachments/assets/26748395-add2-4757-8d9f-f4d24fbee5a7" /> | <img width="420" alt="MB-3D-BOTTOM" src="https://github.com/user-attachments/assets/2181a578-62b7-43b6-971a-71c5a09fe83f" /> |

<p align="center">
  <img width="600" alt="PCB Layout" src="https://github.com/user-attachments/assets/003e6266-b337-4695-b552-10e06c502872" />
  <br />
  <em>PCB 本体 Layout</em>
</p>

### 机械结构

<p align="center">
  <img width="600" alt="3D 结构框架透视图" src="https://github.com/user-attachments/assets/1d9fed10-6077-42a5-a9cf-18adb0dea282" />
  <br />
  <em>3D 结构框架透视图</em>
</p>