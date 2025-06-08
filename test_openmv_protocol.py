#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenMV循迹协议测试脚本
用于测试OpenMV发送的数据格式是否与STM32解析一致
"""

import serial
import time
import struct

class YbProtocolTest:
    """YbProtocol测试类"""
    
    def __init__(self):
        self.ID_COLOR = 1  # 颜色检测功能ID
        
    def get_color_data(self, x, y, w, h):
        """构造颜色检测数据包"""
        # 构造数据字符串: x,y,w,h
        data_str = f"{x},{y},{w},{h}"
        
        # 计算总长度 (包括协议头尾)
        total_len = len(data_str) + 6  # $length,1, + data_str + #
        
        # 构造完整数据包
        packet = f"${total_len},{self.ID_COLOR},{data_str}#"
        
        return packet.encode('utf-8')

def test_protocol():
    """测试协议数据包构造"""
    print("OpenMV循迹协议测试")
    print("=" * 50)
    
    pto = YbProtocolTest()
    
    # 测试数据
    test_cases = [
        {"center_x": 320, "angle": 0, "control_output": 50, "found": 1},
        {"center_x": 280, "angle": -15, "control_output": 80, "found": 1},
        {"center_x": 360, "angle": 10, "control_output": 30, "found": 1},
        {"center_x": 320, "angle": 0, "control_output": 0, "found": 0},
    ]
    
    for i, case in enumerate(test_cases, 1):
        print(f"\n测试用例 {i}:")
        print(f"  输入: center_x={case['center_x']}, angle={case['angle']}, control_output={case['control_output']}, found={case['found']}")
        
        # 构造数据包 (OpenMV发送格式)
        packet = pto.get_color_data(
            case['center_x'],
            case['angle'], 
            int(abs(case['control_output'])),
            case['found']
        )
        
        print(f"  数据包: {packet}")
        print(f"  解码: {packet.decode('utf-8')}")
        
        # 模拟STM32解析
        parse_result = simulate_stm32_parse(packet.decode('utf-8'))
        print(f"  STM32解析结果: {parse_result}")

def simulate_stm32_parse(packet_str):
    """模拟STM32的数据解析过程"""
    try:
        # 检查协议头尾
        if not (packet_str.startswith('$') and packet_str.endswith('#')):
            return "错误: 协议头尾不匹配"
        
        # 去除协议头尾
        data_str = packet_str[1:-1]
        
        # 按逗号分割
        fields = data_str.split(',')
        
        if len(fields) < 6:
            return f"错误: 字段数量不足 ({len(fields)}/6)"
        
        # 解析字段
        length = int(fields[0])
        func_id = int(fields[1])
        line_x = int(fields[2])
        line_y = int(fields[3])  # 实际是角度
        line_angle = int(fields[3])  # 角度
        control_output = int(fields[4])
        found = int(fields[5])
        
        # 验证长度
        if length != len(packet_str):
            return f"错误: 长度不匹配 (声明:{length}, 实际:{len(packet_str)})"
        
        # 验证功能ID
        if func_id != 1:
            return f"错误: 功能ID不匹配 (期望:1, 实际:{func_id})"
        
        return {
            "length": length,
            "func_id": func_id,
            "line_x": line_x,
            "line_y": line_y,
            "line_angle": line_angle,
            "control_output": control_output,
            "line_detected": found > 0,
            "valid": True
        }
        
    except Exception as e:
        return f"解析错误: {str(e)}"

def test_serial_communication(port="COM3", baudrate=115200):
    """测试串口通信"""
    print(f"\n串口通信测试 (端口: {port}, 波特率: {baudrate})")
    print("=" * 50)
    
    try:
        # 打开串口
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"串口 {port} 打开成功")
        
        pto = YbProtocolTest()
        
        # 发送测试数据
        test_data = [
            (320, 0, 50, 1),    # 中心位置，直线
            (280, -15, 80, 1),  # 左偏，需要右转
            (360, 10, 30, 1),   # 右偏，需要左转
            (320, 0, 0, 0),     # 未检测到线条
        ]
        
        for i, (x, angle, output, found) in enumerate(test_data, 1):
            packet = pto.get_color_data(x, angle, output, found)
            
            print(f"\n发送测试数据 {i}: {packet.decode('utf-8')}")
            ser.write(packet)
            
            # 等待响应
            time.sleep(0.1)
            
            # 读取响应
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                print(f"收到响应: {response}")
            else:
                print("无响应")
            
            time.sleep(1)
        
        ser.close()
        print("\n串口测试完成")
        
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except Exception as e:
        print(f"测试错误: {e}")

if __name__ == "__main__":
    # 运行协议测试
    test_protocol()
    
    # 询问是否进行串口测试
    print("\n" + "=" * 50)
    choice = input("是否进行串口通信测试? (y/n): ")
    
    if choice.lower() == 'y':
        port = input("请输入串口号 (默认COM3): ").strip()
        if not port:
            port = "COM3"
        
        try:
            baudrate = int(input("请输入波特率 (默认115200): ") or "115200")
        except ValueError:
            baudrate = 115200
        
        test_serial_communication(port, baudrate)
    
    print("\n测试完成!")