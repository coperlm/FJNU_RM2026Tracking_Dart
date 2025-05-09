# FJNU_RM2026 智能追踪飞镖系统

# 声明：此版本属于研发自用调试阶段，暂未正式开源；如其他队伍使用出现问题，请自行承担风险。

![版本](https://img.shields.io/badge/版本-v1.2-blue)
![平台](https://img.shields.io/badge/平台-STM32F401-green)
![更新日期](https://img.shields.io/badge/更新日期-2025--04--21-orange)

## 项目概述

本项目是福建师范大学机器人团队2026年开发的智能追踪飞镖系统，基于STM32F401单片机开发。系统通过舵机控制飞行姿态，利用涵道电机提供推力，能够实现自主的目标识别、追踪和攻击功能。设计目标是在机器人竞赛中提供高精度、快速响应的自动化打击系统。

## 硬件组成

- **主控板**：STM32F401单片机（72MHz主频，512KB Flash，96KB RAM）
- **执行机构**：
  - 2路舵机（左右舵翼控制）- MG996R或同等规格
  - 2路涵道电机（提供飞行推力）- 最大推力根据飞镖重量选配
- **传感器**：
  - MPU6050姿态传感器（三轴加速度计+三轴陀螺仪）
  - 可选：视觉模块（获取目标位置信息）
- **电源系统**：
  - 锂电池供电（推荐11.1V 3S锂电）
  - 电压转换模块（为不同部件提供所需电压）

## 软件架构

系统基于HAL库开发，主要包含以下几个模块：
1. **姿态控制模块**（`attitude_control.c/h`）：负责飞镖的姿态稳定与调整
2. **飞镖控制主模块**（`dart_control.c/h`）：整体控制逻辑与状态管理
3. **PID控制器模块**（`pid_controller.c/h`）：提供精准的控制算法
4. **PWM输出控制模块**（`pwm_control.c/h`）：控制舵机和电机
5. **OLED显示模块**（`oled.c/h`）：实时显示系统状态与参数

## 功能特点

- **多状态控制**：系统支持初始化、空闲、目标追踪和攻击四种状态
- **PID姿态控制**：采用PID算法实现稳定的姿态控制，快速响应姿态变化
- **目标追踪**：能够自动追踪并锁定识别到的目标，最大跟踪速度可达60°/s
- **参数可调**：通过串口或OLED界面，控制参数易于调节，适应不同飞行环境
- **实时监控**：可通过串口查看实时飞行数据，便于调试与优化
- **低功耗设计**：在未启动攻击模式时，系统能耗较低，延长续航时间

## 技术参数

- **舵机控制**：
  - PWM频率：50Hz
  - 控制范围：1000-2000μs
  - 响应时间：<0.1s
- **电机控制**：
  - PWM频率：50Hz
  - 控制范围：1000-2000μs
  - 最大推力：根据配置选择
- **姿态控制范围**：
  - 横滚角（Roll）：±45°
  - 俯仰角（Pitch）：±45°
  - 偏航角（Yaw）：±90°
- **系统响应**：
  - 姿态调整响应：<0.2s
  - 目标追踪响应：<0.5s

## 开发环境

- **IDE**：Keil MDK 5.38
- **STM32库**：STM32CubeF4 V1.27.1
- **操作系统**：Windows 11
- **烧录工具**：ST-Link V2

## 项目结构

```
Core/
  ├─Inc/                   # 头文件目录
  │  ├─attitude_control.h  # 姿态控制模块头文件
  │  ├─dart_control.h      # 飞镖控制主模块头文件
  │  ├─pid_controller.h    # PID控制器模块头文件
  │  ├─pwm_control.h       # PWM控制模块头文件
  │  ├─oled.h             # OLED显示模块头文件
  │  └─...                 # 其他HAL配置头文件
  └─Src/                   # 源代码目录
     ├─attitude_control.c  # 姿态控制模块实现
     ├─dart_control.c      # 飞镖控制主模块实现
     ├─pid_controller.c    # PID控制器模块实现
     ├─pwm_control.c       # PWM控制模块实现
     ├─oled.c             # OLED显示模块实现
     ├─main.c              # 主程序入口
     └─...                 # 其他HAL配置源文件
Drivers/                   # STM32 HAL驱动目录
MDK-ARM/                   # Keil MDK项目文件目录
```

## 主要更新日志

### v1.2 (2025.04.21)
- 优化PID参数，提高姿态稳定性
- 增加OLED显示功能，方便调试与监控
- 改进目标追踪算法，提高锁定速度与精度
- 增加电池电量监测功能

### v1.1 (2025.03.15)
- 修复舵机抖动问题
- 增强抗干扰能力
- 优化电机控制逻辑，降低功耗

### v1.0 (2025.02.01)
- 项目初始版本发布
- 实现基础追踪与打击功能

## 开发团队

福建师范大学机器人团队-飞镖组

## 联系方式

- **项目负责人**：[团队负责人]
- **技术支持**：[技术支持邮箱/联系方式]
- **项目仓库**：[若有代码仓库，可填写链接]

## 许可证

本项目采用MIT许可证。详见LICENSE文件。