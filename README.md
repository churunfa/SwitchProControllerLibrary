# Switch Pro Controller Library 🎮

[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux%20%7C%20macOS-blue)]()
[![Language](https://img.shields.io/badge/Language-Python%20%2F%20C%2B%2B-green)]()

**SwitchProControllerLibrary** 是一个运行在PC端的控制库。

它旨在配合硬件固件 [SwitchProControllerEsp32S3](https://github.com/churunfa/SwitchProControllerEsp32S3) 使用。通过串口 (Serial) 通讯，本库允许开发者使用 C++ 编写脚本，直接控制 Nintendo Switch 主机。

## 🏗️ 系统架构

```mermaid
graph LR
    PC[电脑 / 树莓派] -- USB-TTL (串口指令) --> ESP32[ESP32-S3 转换器]
    ESP32 -- USB HID (手柄信号) --> Switch[Switch 主机]
```

## 🛠️ 硬件要求

* **开发板**：ESP32-S3 (支持原生 USB)
* **配套硬件**：请参考 [SwitchProControllerEsp32S3](https://github.com/churunfa/SwitchProControllerEsp32S3/tree/main) 部署

## 📦 安装指南

1.  下载本项目代码：
    ```bash
    git clone https://github.com/churunfa/SwitchProControllerLibrary.git
    ```
2.  将 `lib` 文件夹放置在您的项目目录下。
3.  按照 PC -> 开发板COM口 -> 开发板USB口 -> Switch 进行连接即可通过lib库操作Switch

## 功能按键

| 分类 | 按键 |
|------|------|
| 动作键 | BUTTON_A / BUTTON_B / BUTTON_X / BUTTON_Y |
| 功能键 | BUTTON_HOME / BUTTON_CAPTURE / BUTTON_PLUS (+) / BUTTON_MINUS (-) |
| 肩键 / 扳机 | BUTTON_L / BUTTON_R / BUTTON_ZL / BUTTON_ZR / 
| D-Pad（方向键） | DPAD_UP / DPAD_DOWN / DPAD_LEFT / DPAD_RIGHT |
| 摇杆按键 | BUTTON_THUMB_L (左键按下) / BUTTON_THUMB_R (右键按下) |

## 接口
```
void pressButton(ButtonType button); // 按下指定按键(入参参考上述功能按键)
void releaseButton(ButtonType button); // 松开指定按键
// 左摇杆，x,y -> 0~4096; 居中：2048
void moveLeftAnalog(uint16_t x, uint16_t y); // 移动左摇杆
void resetLeftAnalog(); // 释放左摇杆（居中）
// 右摇杆，x,y -> 0~4096; 居中：2048
void moveRightAnalog(uint16_t x, uint16_t y); // 移动右摇杆
void resetRightAnalog(); // 释放右摇杆
// 体感角速度和加速度
void setIMU(int16_t accX, int16_t accY, int16_t accZ, int16_t gyroX, int16_t gyroY, int16_t gyroZ); // 模拟陀螺仪
void resetIMU();// 重置陀螺仪
void resetAll(); // 释放所有按键
void sendReport(); // 手动发送报告
```

设置按键状态后会以5ms/次的速度同步到开发板，开发版也会以5ms/次的速度同步到Switch。前者可以通过手动执行sendReport()进行立刻同步。


