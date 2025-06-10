# Vision Tracker 使用说明

## 概述

本项目已成功整合了循迹模式和物体追踪模式的视觉功能，通过OpenMV运行 `vision_tracker.py` 脚本与STM32进行通信。

## 文件说明

### OpenMV端
- `vision_tracker.py`: 融合了循迹和物体追踪功能的主程序
  - 模式1: 循迹模式 (基于原 `openmv_line_tracking.py`)
  - 模式2: 物体追踪模式 (基于原 `color_detect.py`)

### STM32端修改
- `k230_vision.h/c`: 适配新的数据协议格式
- `balance_control.c`: 更新循迹和物体追踪控制算法
- `bluetooth.c/h`: 添加颜色设置命令支持

## 通信协议

### OpenMV -> STM32 (UART2)

#### 循迹模式数据格式
```
$length,1,center_x,angle,speed_percentage,found#
```
- `center_x`: 线条中心X坐标 (0-640)
- `angle`: 线条角度 (-90 到 90度)
- `speed_percentage`: 速度百分比 (0-100)
- `found`: 检测状态 (0/1)

#### 物体追踪模式数据格式
```
$length,1,center_x,center_y,width,found#
```
- `center_x`: 物体中心X坐标 (0-640)
- `center_y`: 物体中心Y坐标 (0-480)
- `width`: 物体宽度
- `found`: 检测状态 (0/1)

### STM32 -> OpenMV (UART2)

#### 模式切换命令
- `MODE_0\r\n`: 空闲模式
- `MODE_1\r\n`: 循迹模式
- `MODE_2\r\n`: 物体追踪模式

#### 颜色设置命令
- `COLOR_RED\r\n`: 设置追踪红色
- `COLOR_GREEN\r\n`: 设置追踪绿色
- `COLOR_BLUE\r\n`: 设置追踪蓝色

## 蓝牙控制命令

### 基本控制
- `START`: 开始平衡控制
- `STOP`: 停止平衡控制
- `RESET`: 重置系统

### 视觉模式控制
- `LINE`: 启用循迹模式
- `TRACK`: 启用物体追踪模式
- `VOFF`: 关闭视觉模式

### 颜色设置
- `COLOR RED`: 设置追踪红色物体
- `COLOR GREEN`: 设置追踪绿色物体
- `COLOR BLUE`: 设置追踪蓝色物体

### 运动控制
- `FORWARD [speed]`: 前进
- `BACKWARD [speed]`: 后退
- `LEFT [rate]`: 左转
- `RIGHT [rate]`: 右转

## 使用步骤

1. **硬件连接**
   - OpenMV通过UART2与STM32连接
   - 蓝牙模块通过UART3与STM32连接

2. **启动系统**
   - 将 `vision_tracker.py` 上传到OpenMV并运行
   - 启动STM32程序
   - 通过蓝牙发送 `GO` 命令建立连接

3. **循迹模式使用**
   ```
   LINE          # 启用循迹模式
   START         # 开始平衡控制
   ```

4. **物体追踪模式使用**
   ```
   COLOR RED     # 设置追踪红色物体
   TRACK         # 启用物体追踪模式
   START         # 开始平衡控制
   ```

## 控制参数调整

### 循迹参数
- 基础速度: 0.25 m/s
- 角度速度因子: 根据线条角度动态调整
- 转向速度因子: 根据转向误差动态调整

### 物体追踪参数
- 基础速度: 0.25 m/s
- 距离因子: 根据物体宽度估算距离
- 转向速度因子: 根据转向误差动态调整

## 故障排除

1. **无视觉数据**
   - 检查UART2连接
   - 确认OpenMV正在运行 `vision_tracker.py`
   - 检查串口波特率设置

2. **循迹效果不佳**
   - 调整线条颜色阈值
   - 检查光照条件
   - 调整PID参数

3. **物体追踪不稳定**
   - 确认目标颜色设置正确
   - 调整颜色阈值范围
   - 检查目标物体大小和距离

## 技术特性

- **双模式支持**: 循迹和物体追踪可动态切换
- **增强PID控制**: 支持积分限幅和输出限幅
- **自适应速度**: 根据检测状态和误差动态调整速度
- **低通滤波**: 减少转向控制抖动
- **渐进停止**: 检测丢失时平滑减速停止