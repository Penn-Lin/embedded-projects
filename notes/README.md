# 学习笔记

五个项目的逐行代码精读页面。每个页面是**完全自包含的单文件 HTML**（无外部依赖、无 CDN），直接用浏览器打开即可。

| 页面 | 内容 |
|---|---|
| [index.html](index.html) | 总览：五个项目一条线 |
| [01-pic-auto-car.html](01-pic-auto-car.html) | `task1.c` 逐行精读 · PIC16F877A 自动行驶小车 |
| [02-esp32-speaker.html](02-esp32-speaker.html) | `Sandspeaker.ino` · ESP32 磁流体视觉音响 |
| [03-fpga-dual-car.html](03-fpga-dual-car.html) | 六个 VHDL 模块 · FPGA 双车系统 |
| [04-gas-sampling.html](04-gas-sampling.html) | `BTcontrol_v2.ino` · 气体检测采样单元 |
| [05-oled-buzzer.html](05-oled-buzzer.html) | `FinalVersion_oled_buzzer_switch.ino` · OLED 装置 |

## 为什么单独放

这些页面不是项目文档，而是**为面试准备的自问自答**：每一处关键代码后面跟着「为什么这样写」「如果换个做法会怎样」。项目 README 讲的是「做了什么」，这些页面讲的是「为什么这么做」。

PIC 小车的精读页在 [`../01-pic-autonomous-car/docs/code-walkthrough.html`](../01-pic-autonomous-car/docs/code-walkthrough.html)。

## 在线版

推送到 GitHub 后可在 Settings → Pages 中选择 `main` 分支的 `/ (root)` 目录发布，访问 `https://<用户名>.github.io/<仓库名>/notes/`。
