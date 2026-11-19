> 🚧 **开发中** —— 本项目是「古明地觉 cos」相关装置的一部分，仍在持续开发中，电路、固件与文档可能随时调整。

> 🌟 **二次开发声明**
> 「觉之瞳」的概念与早期机械设计来自 B 站 UP 主 **牧鈴華SUZUKA** 的开源作品：
> - 原视频：[「开源」机械觉之瞳](https://www.bilibili.com/video/BV1rN1gYJE3K/)
> - 原设计版权归原作者所有。本仓库是在其设计基础上的二次开发（固件 / 结构 / 功能扩展），并非原创设计。

---
# esp32s3_face_follow

觉之瞳 v2 的"人脸跟随"固件骨架：OV2640 → ESP-DL 人脸检测 → 增量式视觉伺服 → MG90S ×2(/3)。

## 1. 前置

- 硬件：ESP32-S3-WROOM-1 **N16R8**（8MB PSRAM 必须）、OV2640 24P 模组、3× MG90S、5V/3A 降压、3.3V LDO。
- 软件：ESP-IDF **v5.1 以上**（Windows 用官方安装器 `esp-idf-tools-setup`）。
- ⚠️ 工程路径**必须纯 ASCII**。本工作区在中文路径下，构建请走 junction：

  ```powershell
  cd D:\mcu\workspace\esp32s3_face_follow
  ```

## 2. 编译烧写

```powershell
cd D:\mcu\workspace\esp32s3_face_follow
idf.py set-target esp32s3
idf.py build
idf.py -p COM5 flash monitor
```

首次 `build` 会按 `main/idf_component.yml` 自动拉取 `esp32-camera` 和 `esp-dl`（需要联网）。

## 3. 接线

| 信号 | GPIO |
|---|---|
| OV2640 SDA / SCL | 4 / 5 |
| OV2640 VSYNC / HREF / PCLK / XCLK | 6 / 7 / 13 / 15 |
| OV2640 D0–D7 | 11, 9, 8, 10, 12, 18, 17, 16 |
| 舵机 yaw / pitch / iris | 38 / 39 / 40 |

- 摄像头 XCLK 用 `LEDC_TIMER_0/CHANNEL_0`，舵机用 `LEDC_TIMER_1/CHANNEL_1..3`，**不要改回 TIMER_0**。
- GPIO 33–37 被 Octal PSRAM 占用，不要接任何东西。

## 4. 参数在哪改

`main/app_main.cpp` 顶部：

| 参数 | 含义 | 调整方法 |
|---|---|---|
| `YAW_SIGN` / `PITCH_SIGN` | 方向 | 开环测试后取 ±1 |
| `YAW_MIN/MAX`、`PITCH_MIN/MAX` | 行程限位 | 按机构实际，别顶死 |
| `KP` | 跟随力度 | 从小到大加，摆了就减 |
| `DEADZONE` | 死区 | 略大于舵机回差 |
| `MAX_RATE_DPS` | 最大角速度 | 防甩头 |
| `SMOOTH` | 输出平滑 | 越小越稳越迟钝 |

## 5. 调试顺序

1. 只接摄像头：串口应打印 `xx fps, face=0/1`；
2. 串口打印人脸框，确认坐标；
3. 舵机开环（把 `control_task` 里改成固定角度扫），确认方向与限位；
4. 单轴闭环（先 yaw）→ 双轴；
5. 装壳复测：排线不扯、不刮、WiFi 不掉。

## 6. 版本注意

- `dl::image::img_t` 的字段以 `esp-dl/include/dl_image_define.hpp` 为准；
- 若 `detector->run(img)` 编译报参数类型不符，把 `img` 改成 `const dl::image::img_t` 传引用即可；
- 若想换成 RGB888：`pixel_format = PIXFORMAT_RGB888`、`pix_type = DL_IMAGE_PIX_TYPE_RGB888`，帧率会掉一半。

## 7. 与本机现有工具链的关系

本工作区已装 STM32 / Pico 工具链（`D:\mcu`），**未装 ESP-IDF**。
ESP-IDF 与它们不冲突，独立安装到 `C:\esp\` 即可，但同样遵守"纯 ASCII 路径"。
