# 嵌入式项目集 · Embedded Projects

集美大学 通信工程 · 五个从 8 位裸机到 FPGA 的硬件项目。

代码、报告与文档均来自实际交付版本。团队成员项目在各自 README 中标注了本人分工。

## 项目索引

| # | 项目 | 平台 | 关键词 | 级别 |
|---|---|---|---|---|
| 01 | [PIC 自动行驶小车](01-pic-autonomous-car/) | PIC16F877A · 4 MHz 裸机 | 陀螺仪 P 控制 · 8 路灰度 · 路线表状态机 | ★★★ |
| 02 | [ESP32 磁流体视觉蓝牙音响](02-esp32-ferrofluid-speaker/) | ESP32 双核 · LVGL | A2DP 音频可视化 · 全流程自研硬件 | ★★★ |
| 03 | [FPGA 双车循迹与跟随](03-fpga-dual-car/) | Cyclone IV · VHDL | 五模块顶层例化 · 红外握手 · 超声四档跟随 | ★★★ |
| 04 | [气体检测采样控制单元](04-arduino-gas-sampling/) | Arduino Nano | I2C + CRC8 校验 · 非阻塞采样 · 蓝牙指令集 | ★★ |
| 05 | [AVR OLED 显示与蜂鸣器](05-avr-oled-buzzer/) | Arduino UNO · AVR | PROGMEM 解决 SRAM 溢出 · 四级按键优先级 | ★★ |

## 一句话概览

**01 · PIC 自动行驶小车** —— 用 4 MHz 单片机从寄存器开始驱动自动行驶小车，场地只在两段圆弧处有黑线，直线段靠陀螺仪航向闭环。四档任务实测 3 / 12 / 13 / 56 秒，时限分别为 15 / 30 / 40 / 100 秒。

**02 · ESP32 磁流体视觉蓝牙音响** —— 个人项目，从原理图、双层 PCB、打样焊接到固件全部独立完成。ESP32 双核把蓝牙音频接收与屏幕渲染分开，音频频段能量实时驱动磁流体动画，8 bit 色深把全屏缓冲从 115 KB 压到 58 KB，实测约 20 FPS。

**03 · FPGA 双车系统** —— 50 MHz 单时钟域内用五个 VHDL 模块搭出双车协同：领车红外脉冲启动，跟车按超声波距离四档调速，圈数计数用「15 ms 确认 + 1 s 冷却 + 圈内锁存」保证四圈不串圈。

**04 · 气体检测采样单元** —— Arduino Nano 驱动气泵抽气，I2C 数字流量计自写 CRC8 校验与线性标定，经蓝牙每 2 秒回传流量与温湿度，并支持手机下发指令控制紫光灯与气泵。

**05 · OLED 装置** —— 大一第一个完整项目。把 1 KB 全屏位图与曲谱全部移入 Flash，解决 AVR 仅 2 KB SRAM 的溢出问题。

## 技术栈

`C` `C++` `VHDL` `汇编启动文件` · `PIC16F877A` `Arduino Nano/UNO` `ESP32` `Cyclone IV` · `MPLAB X` `Arduino IDE` `Quartus` `嘉立创 EDA` · `UART` `I2C` `PWM` `定时器中断` `A2DP` `蓝牙串口`

## 目录约定

每个项目统一结构：

```
NN-project-name/
├── src/          源码与工程配置文件
├── firmware/     可直接烧录的产物（.hex / .pof / .ino）
├── hardware/     原理图与 PCB 工程（仅 02）
├── docs/         报告、任务书、图片
└── README.md
```

## 学习笔记

[`notes/`](notes/) 下有五个项目的逐行代码精读页面（HTML，可直接打开或看 [在线版](https://penn-lin.github.io/embedded-projects/notes/)）。

## 许可

代码采用 [MIT License](LICENSE)。课程设计报告与文档仅作展示，请勿直接抄用。
