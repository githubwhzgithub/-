# OpenMV循迹追踪算法使用指南
# OpenMV Line Tracking Algorithm User Guide

## 概述 / Overview

本项目提供了两个基于OpenMV和MicroPython的循迹追踪算法实现：
This project provides two line tracking algorithm implementations based on OpenMV and MicroPython:

1. **完整版循迹算法** (`openmv_line_tracking.py`) - 功能完整的专业级实现
2. **简化版循迹算法** (`simple_line_tracking.py`) - 适合初学者的简化版本

## 硬件要求 / Hardware Requirements

- OpenMV Cam H7 或兼容设备 / OpenMV Cam H7 or compatible device
- STM32微控制器 (用于接收控制命令) / STM32 microcontroller (for receiving control commands)
- 串口连接线 / UART connection cables
- 循迹小车底盘 / Line tracking robot chassis
- 黑色胶带或彩色线条 / Black tape or colored lines

## 软件要求 / Software Requirements

- OpenMV IDE 4.0+ 
- MicroPython 固件 / MicroPython firmware
- STM32开发环境 (Keil, STM32CubeIDE等) / STM32 development environment

## 算法特性对比 / Algorithm Feature Comparison

| 特性 / Feature | 完整版 / Full Version | 简化版 / Simple Version |
|----------------|----------------------|------------------------|
| 多颜色支持 / Multi-color | ✅ 5种颜色 / 5 colors | ❌ 仅黑色 / Black only |
| PID控制 / PID Control | ✅ 完整PID / Full PID | ❌ 简单阈值 / Simple threshold |
| 角度检测 / Angle Detection | ✅ 支持 / Supported | ❌ 不支持 / Not supported |
| 置信度计算 / Confidence | ✅ 支持 / Supported | ❌ 不支持 / Not supported |
| 串口命令 / UART Commands | ✅ 丰富命令 / Rich commands | ✅ 基础命令 / Basic commands |
| LED指示 / LED Indicators | ✅ 三色指示 / 3-color | ✅ 三色指示 / 3-color |
| 代码复杂度 / Complexity | 高 / High | 低 / Low |
| 适用场景 / Use Case | 专业应用 / Professional | 学习入门 / Learning |

## 快速开始 / Quick Start

### 1. 简化版使用 / Simple Version Usage

```python
# 1. 将 simple_line_tracking.py 上传到OpenMV
# 2. 连接硬件
# 3. 运行程序
exec(open('simple_line_tracking.py').read())
```

**硬件连接 / Hardware Connection:**
```
OpenMV    STM32
P4 (TX) → PA10 (RX)
P5 (RX) → PA9  (TX)
GND     → GND
```

### 2. 完整版使用 / Full Version Usage

```python
# 1. 将 openmv_line_tracking.py 上传到OpenMV
# 2. 根据需要调整配置参数
# 3. 运行程序
exec(open('openmv_line_tracking.py').read())
```

## 配置参数说明 / Configuration Parameters

### 完整版配置 / Full Version Configuration

```python
class Config:
    # 颜色阈值 / Color Thresholds
    COLOR_THRESHOLDS = {
        'red': (30, 100, 15, 127, 15, 127),      # 红色
        'green': (30, 100, -64, -8, -32, 32),   # 绿色
        'blue': (0, 30, 0, 64, -128, 0),        # 蓝色
        'black': (0, 30, -20, 20, -20, 20),     # 黑色
        'white': (80, 100, -20, 20, -20, 20)    # 白色
    }
    
    # PID参数 / PID Parameters
    PID_KP = 0.8  # 比例系数，控制响应速度
    PID_KI = 0.1  # 积分系数，消除稳态误差
    PID_KD = 0.2  # 微分系数，减少超调
    
    # 检测参数 / Detection Parameters
    MIN_BLOB_AREA = 100      # 最小色块面积
    MAX_BLOB_AREA = 10000    # 最大色块面积
    ROI_Y_START = 120        # 检测区域起始Y坐标
    ROI_HEIGHT = 120         # 检测区域高度
```

### 简化版配置 / Simple Version Configuration

```python
class SimpleConfig:
    # 黑线阈值 / Black Line Threshold
    BLACK_THRESHOLD = (0, 30, -20, 20, -20, 20)
    
    # 检测区域 / Detection Area
    ROI_Y = 160      # 检测区域Y坐标
    ROI_HEIGHT = 80  # 检测区域高度
    
    # 转向阈值 / Turn Threshold
    TURN_THRESHOLD = 30  # 像素偏移阈值
```

## 串口通信协议 / UART Communication Protocol

### 完整版协议 / Full Version Protocol

**发送格式 / Send Format:**
```
LINE:<center_x>,<angle>,<control_output>,<found>
例如 / Example: LINE:160,15,25,1
```

**接收命令 / Receive Commands:**
```
COLOR_RED    - 设置追踪红色 / Set tracking to red
COLOR_GREEN  - 设置追踪绿色 / Set tracking to green
COLOR_BLUE   - 设置追踪蓝色 / Set tracking to blue
COLOR_BLACK  - 设置追踪黑色 / Set tracking to black
COLOR_WHITE  - 设置追踪白色 / Set tracking to white
RESET        - 重置PID控制器 / Reset PID controller
STATUS       - 查询当前状态 / Query current status
```

### 简化版协议 / Simple Version Protocol

**发送格式 / Send Format:**
```
<DIRECTION>:<center_x>
例如 / Example: FORWARD:160, LEFT:120, RIGHT:200, STOP:160
```

## LED指示说明 / LED Indicator Description

| LED颜色 / LED Color | 状态 / Status | 含义 / Meaning |
|-------------------|--------------|---------------|
| 🔴 红色 / Red | 亮 / ON | 未检测到线条 / Line not detected |
| 🟢 绿色 / Green | 亮 / ON | 检测到线条 / Line detected |
| 🔵 蓝色 / Blue | 亮 / ON | 需要转向 / Need to turn |

## 调试和优化 / Debugging and Optimization

### 1. 颜色阈值调整 / Color Threshold Adjustment

使用OpenMV IDE的阈值编辑器来获取准确的颜色阈值：
Use OpenMV IDE's threshold editor to get accurate color thresholds:

```python
# 在OpenMV IDE中运行此代码来调试阈值
# Run this code in OpenMV IDE to debug thresholds
import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

while(True):
    img = sensor.snapshot()
    # 使用IDE的阈值编辑器选择颜色
    # Use IDE's threshold editor to select colors
```

### 2. PID参数调优 / PID Parameter Tuning

**调优步骤 / Tuning Steps:**

1. **设置Kp** - 从小值开始，逐渐增加直到系统响应足够快
2. **设置Kd** - 添加微分项减少超调和振荡
3. **设置Ki** - 最后添加积分项消除稳态误差

```python
# 保守调优 / Conservative tuning
PID_KP = 0.5
PID_KI = 0.05
PID_KD = 0.1

# 激进调优 / Aggressive tuning
PID_KP = 1.2
PID_KI = 0.2
PID_KD = 0.3
```

### 3. 性能优化建议 / Performance Optimization Tips

- **减小图像分辨率** - 使用QQVGA (160x120) 提高帧率
- **优化ROI区域** - 只检测必要的图像区域
- **调整曝光设置** - 固定曝光值避免自动调整延迟
- **使用合适的线条宽度** - 推荐15-25mm宽度的线条

## 常见问题解决 / Troubleshooting

### 问题1：检测不到线条 / Issue 1: Cannot detect lines

**解决方案 / Solutions:**
- 检查颜色阈值设置
- 确保光照条件稳定
- 调整ROI区域位置
- 检查线条宽度和对比度

### 问题2：检测不稳定 / Issue 2: Unstable detection

**解决方案 / Solutions:**
- 增加最小色块面积阈值
- 使用merge=True合并相邻色块
- 调整PID参数减少振荡
- 改善环境光照条件

### 问题3：转向响应慢 / Issue 3: Slow turning response

**解决方案 / Solutions:**
- 增加Kp值提高响应速度
- 减小控制输出的死区
- 检查机械结构是否灵活
- 优化算法执行频率

## STM32接收端示例代码 / STM32 Receiver Example Code

```c
// STM32端串口接收处理示例
// STM32 UART receive handling example

#include "usart.h"
#include "string.h"
#include "stdio.h"

char uart_buffer[100];
int center_x, angle, control_output, found;

void process_line_data(char* data) {
    if (strncmp(data, "LINE:", 5) == 0) {
        // 解析完整版数据格式
        sscanf(data + 5, "%d,%d,%d,%d", 
               &center_x, &angle, &control_output, &found);
        
        if (found) {
            // 根据control_output控制电机
            motor_control(control_output);
        } else {
            // 停止或搜索模式
            motor_stop();
        }
    }
    else if (strstr(data, "FORWARD") != NULL) {
        // 简化版前进命令
        motor_forward();
    }
    else if (strstr(data, "LEFT") != NULL) {
        // 简化版左转命令
        motor_turn_left();
    }
    else if (strstr(data, "RIGHT") != NULL) {
        // 简化版右转命令
        motor_turn_right();
    }
    else if (strstr(data, "STOP") != NULL) {
        // 停止命令
        motor_stop();
    }
}

// 在主循环中调用
void main_loop() {
    if (HAL_UART_Receive(&huart1, (uint8_t*)uart_buffer, 
                        sizeof(uart_buffer), 10) == HAL_OK) {
        process_line_data(uart_buffer);
    }
}
```

## 扩展功能 / Extended Features

### 1. 多线条追踪 / Multi-line Tracking

可以修改算法支持同时追踪多条线：
The algorithm can be modified to support tracking multiple lines simultaneously:

```python
def find_multiple_lines(img, max_lines=3):
    """
    查找多条线 / Find multiple lines
    """
    blobs = img.find_blobs([threshold], merge=False)
    lines = []
    
    for blob in sorted(blobs, key=lambda b: b.area(), reverse=True)[:max_lines]:
        if blob.area() > MIN_AREA:
            lines.append({
                'center_x': blob.cx(),
                'center_y': blob.cy(),
                'angle': blob.rotation_deg(),
                'area': blob.area()
            })
    
    return lines
```

### 2. 路径记录 / Path Recording

添加路径记录功能用于分析和回放：
Add path recording functionality for analysis and playback:

```python
class PathRecorder:
    def __init__(self):
        self.path_points = []
        self.start_time = time.ticks_ms()
    
    def record_point(self, x, y, timestamp=None):
        if timestamp is None:
            timestamp = time.ticks_diff(time.ticks_ms(), self.start_time)
        
        self.path_points.append({
            'x': x,
            'y': y,
            'time': timestamp
        })
    
    def save_path(self, filename):
        # 保存路径数据到文件
        pass
```

### 3. 自适应阈值 / Adaptive Thresholding

实现自动调整颜色阈值的功能：
Implement automatic color threshold adjustment:

```python
def adaptive_threshold(img, roi):
    """
    自适应阈值调整 / Adaptive threshold adjustment
    """
    # 计算ROI区域的统计信息
    stats = img.get_statistics(roi=roi)
    
    # 根据亮度和对比度调整阈值
    l_mean = stats.l_mean()
    l_stdev = stats.l_stdev()
    
    # 动态调整阈值范围
    threshold = (
        max(0, l_mean - 2 * l_stdev),
        min(100, l_mean + 2 * l_stdev),
        -20, 20, -20, 20
    )
    
    return threshold
```

## 许可证 / License

本项目采用MIT许可证，详见LICENSE文件。
This project is licensed under the MIT License, see LICENSE file for details.

## 贡献 / Contributing

欢迎提交问题报告和改进建议！
Welcome to submit issue reports and improvement suggestions!

## 联系方式 / Contact

如有问题，请通过以下方式联系：
If you have any questions, please contact via:

- GitHub Issues
- Email: [your-email@example.com]

---

**注意 / Note:** 使用前请确保已正确配置硬件连接和参数设置。建议先在简单环境下测试算法性能。
Please ensure proper hardware connections and parameter settings before use. It's recommended to test the algorithm performance in a simple environment first.