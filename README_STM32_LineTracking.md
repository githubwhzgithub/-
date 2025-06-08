# STM32平衡车循迹控制系统优化说明

## 概述

本文档详细说明了STM32端平衡车循迹控制系统的优化改进，主要解决了原有系统中转向控制未实现的问题，并与OpenMV端的优化算法进行了深度集成。

## 主要改进内容

### 1. 转向控制系统重构

#### 原有问题
- `BalanceControl_LineTracking`函数只计算了`vision_error_x`，但没有实际的转向控制实现
- 转向PID控制器使用横滚角作为反馈，不适合循迹应用

#### 改进方案
- **直接转向控制**：在视觉模式下，直接使用`vision_error_x`计算转向输出
- **比例控制**：`turn_output = vision_error_x * TURN_PID_KP`
- **输出限幅**：确保转向输出在安全范围内
- **模式切换**：非视觉模式下仍使用横滚角PID控制保持平衡

```c
// 转向控制逻辑
if(BalanceState.vision_mode > 0) {
    // 视觉模式：直接转向控制
    turn_output = BalanceState.vision_error_x * TURN_PID_KP;
    // 输出限幅
    if(turn_output > TURN_PID_MAX_OUTPUT) turn_output = TURN_PID_MAX_OUTPUT;
    if(turn_output < -TURN_PID_MAX_OUTPUT) turn_output = -TURN_PID_MAX_OUTPUT;
} else {
    // 非视觉模式：横滚角平衡控制
    TurnPID.setpoint = 0.0f;
    turn_output = BalanceControl_PID_Update(&TurnPID, BalanceState.roll, dt);
}
```

### 2. 循迹算法优化

#### 转向误差滤波
- **低通滤波**：对原始转向误差进行滤波处理，减少抖动
- **滤波系数**：`filter_alpha = 0.7f`，可根据实际效果调整
- **平滑转向**：提高循迹的稳定性和舒适性

```c
// 转向误差滤波
static float filtered_error_x = 0.0f;
float filter_alpha = 0.7f;
filtered_error_x = filter_alpha * raw_error_x + (1.0f - filter_alpha) * filtered_error_x;
```

#### 智能速度控制
- **OpenMV速度因子**：使用OpenMV发送的动态速度因子
- **转向速度调节**：根据转向幅度进一步调整速度
- **渐进停车**：检测不到线条时逐渐减速，避免急停

```c
// 多层速度控制
float speed_factor = vision_data->line_track.line_angle / 100.0f; // OpenMV速度因子
float turn_speed_factor = 1.0f - 0.5f * fabs(vision_error_x); // 转向速度因子
BalanceState.target_speed = 0.25f * speed_factor * turn_speed_factor;
```

### 3. 与OpenMV协调工作

#### 数据协议
- **位置信息**：`line_x` - 线条中心X坐标
- **速度因子**：`line_angle` - OpenMV计算的动态速度百分比
- **检测状态**：`found` - 线条检测状态标志

#### 数据处理流程
1. **接收OpenMV数据**：通过K230视觉模块获取循迹数据
2. **计算转向误差**：基于线条位置计算标准化误差
3. **滤波处理**：对转向误差进行低通滤波
4. **速度计算**：结合OpenMV速度因子和转向因子
5. **控制输出**：计算电机差速控制信号

## 控制参数说明

### 转向控制参数
```c
#define TURN_PID_KP            20.0f    // 转向比例系数
#define TURN_PID_MAX_OUTPUT    500.0f   // 转向最大输出
```

### 速度控制参数
```c
#define BASE_SPEED             0.25f    // 基础前进速度 (m/s)
#define MIN_SPEED_FACTOR       0.3f     // 最小速度因子
#define TURN_SPEED_REDUCTION   0.5f     // 转向时的速度衰减系数
```

### 滤波参数
```c
#define FILTER_ALPHA           0.7f     // 转向误差滤波系数
#define STOP_DECEL_FACTOR      0.9f     // 停车减速因子
```

## 性能提升

### 1. 转向精度
- **响应性**：直接转向控制，响应更快
- **稳定性**：误差滤波减少抖动
- **准确性**：基于视觉误差的精确控制

### 2. 速度适应性
- **动态调节**：根据线条复杂度自动调速
- **安全性**：转向时自动减速
- **平滑性**：渐进式停车和启动

### 3. 系统鲁棒性
- **容错性**：检测失败时的安全处理
- **模式切换**：视觉/非视觉模式无缝切换
- **参数可调**：关键参数可根据实际情况调整

## 调试和优化建议

### 1. 转向参数调优
- **TURN_PID_KP过大**：转向过于敏感，可能震荡
- **TURN_PID_KP过小**：转向响应慢，跟踪精度差
- **建议范围**：15.0f - 30.0f

### 2. 滤波参数调优
- **filter_alpha过大**：滤波效果弱，可能仍有抖动
- **filter_alpha过小**：过度滤波，响应变慢
- **建议范围**：0.5f - 0.8f

### 3. 速度参数调优
- **BASE_SPEED**：根据场地条件和车辆性能调整
- **TURN_SPEED_REDUCTION**：根据转向稳定性需求调整

### 4. 测试方法
1. **直线测试**：验证直线跟踪的稳定性
2. **弯道测试**：验证转向控制的准确性
3. **复杂路径**：测试综合性能
4. **干扰测试**：验证系统鲁棒性

## 故障排除

### 常见问题

1. **转向不响应**
   - 检查`vision_mode`是否正确设置
   - 验证`vision_error_x`是否有效更新
   - 确认`TURN_PID_KP`参数设置

2. **转向过度敏感**
   - 降低`TURN_PID_KP`值
   - 减小`filter_alpha`增强滤波
   - 检查OpenMV数据质量

3. **速度控制异常**
   - 验证OpenMV速度因子数据
   - 检查速度限制参数
   - 确认电机输出范围

### 调试工具
- **串口监控**：实时查看控制参数
- **蓝牙调试**：无线参数调整
- **LED指示**：状态可视化

## 扩展功能

### 1. 自适应参数
- 根据循迹性能自动调整控制参数
- 学习最优参数组合

### 2. 路径预测
- 基于历史数据预测路径趋势
- 提前调整控制策略

### 3. 多传感器融合
- 结合IMU数据优化控制
- 融合超声波避障功能

## 总结

通过本次优化，STM32端的循迹控制系统实现了：
- **完整的转向控制功能**
- **与OpenMV的深度集成**
- **智能的速度适应机制**
- **稳定的控制性能**

系统现在能够实现精确、稳定、智能的循迹控制，为平衡车的自主导航提供了可靠的基础。