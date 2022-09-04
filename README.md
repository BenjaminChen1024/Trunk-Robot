# Trunk-Robot
Biomimetic elephant trunk robot based on wire pulling technology.

![Image text](/Image/Trunk.jpg)

[![Author](https://img.shields.io/badge/Author-BenjaminChen-blue.svg "Author")](https://github.com/cypypccpy "Author")
[![license](https://img.shields.io/github/license/:user/:repo.svg)](LICENSE)
[![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)
<br></br>

## 内容列表

- [背景](#背景)
- [安装](#安装)
- [用法](#用法)
- [相关参数](#相关参数)
  <br></br>

## 背景

自主研发基于线拉技术的仿生象鼻机器人嵌入式控制代码，基于FreeRTOS实时操作系统与状态机处理。
<br></br>

## 安装

- `STM32CubeMX 6.2.0`
- `STM32CubeIDE 1.4.0`
- `Keil uVision5`
  <br></br>

## 用法

```bash
# 打开 Keil uVision5
# Open 选择 Trunk Robot 文件夹
# 打开 MDK-ARM 文件夹
# 打开 RoboMasterDevelopmentBoardC.uvprojx 文件
```

<br></br>

## 相关参数

### Motor_ID

#### CAN1

| Trunk_Motor1(末端X轴)ID1  | M3508  | 0x201 |
| ------------------------- | ------ | ----- |
| Trunk_Motor2(末端Y轴)ID2  | M3508  | 0x202 |
| Trunk_Motor3(中段左轴)ID3 | M3508  | 0x203 |
| Trunk_Motor4(中断右轴)ID4 | M3508  | 0x204 |
| Yaw_MotorID1              | GM6020 | 0x205 |

#### CAN2

| Chassis_Motor1(左前轮)ID1 | GM6020 | 0x205 |
| ------------------------- | ------ | ----- |
| Chassis_Motor2(左后轮)ID2 | GM6020 | 0x206 |
| Chassis_Motor3(右前轮)ID3 | GM6020 | 0x207 |
| Chassis_Motor4(右后轮)ID4 | GM6020 | 0x208 |
