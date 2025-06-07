#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化的K230通信协议
Simplified K230 Communication Protocol

用于在没有原始协议库时提供基本的通信功能
Provides basic communication functionality when original protocol library is not available

作者 / Author: AI Assistant
版本 / Version: 1.0
日期 / Date: 2024
"""

import struct
import time
from machine import UART

class SimpleProtocol:
    """
    简化的通信协议类 / Simplified communication protocol class
    """
    
    # 功能ID定义 / Function ID definitions
    FUNC_COLOR_DETECT = 0x01      # 颜色检测 / Color detection
    FUNC_LINE_TRACKING = 0x02     # 循迹 / Line tracking
    FUNC_OBJECT_TRACKING = 0x03   # 目标追踪 / Object tracking
    FUNC_NANO_TRACKER = 0x04      # 纳米追踪器 / Nano tracker
    FUNC_STATUS = 0x05            # 状态信息 / Status information
    
    # 数据包格式 / Packet format
    PACKET_HEADER = 0xAA55        # 包头 / Packet header
    PACKET_TAIL = 0x55AA          # 包尾 / Packet tail
    
    def __init__(self):
        """
        初始化协议 / Initialize protocol
        """
        self.sequence = 0  # 序列号 / Sequence number
    
    def _calculate_checksum(self, data):
        """
        计算校验和 / Calculate checksum
        
        Args:
            data: 数据字节 / Data bytes
            
        Returns:
            int: 校验和 / Checksum
        """
        return sum(data) & 0xFF
    
    def _pack_data(self, func_id, data):
        """
        打包数据 / Pack data
        
        Args:
            func_id: 功能ID / Function ID
            data: 数据 / Data
            
        Returns:
            bytes: 打包后的数据 / Packed data
        """
        # 增加序列号 / Increment sequence number
        self.sequence = (self.sequence + 1) & 0xFF
        
        # 构建数据包 / Build packet
        # 格式: Header(2) + Length(1) + FuncID(1) + Seq(1) + Data(N) + Checksum(1) + Tail(2)
        packet_data = struct.pack('<H', self.PACKET_HEADER)  # Header
        packet_data += struct.pack('<B', len(data) + 3)      # Length (FuncID + Seq + Data)
        packet_data += struct.pack('<B', func_id)            # Function ID
        packet_data += struct.pack('<B', self.sequence)      # Sequence
        packet_data += data                                   # Data
        
        # 计算校验和 / Calculate checksum
        checksum = self._calculate_checksum(packet_data[2:])  # 从长度字段开始 / From length field
        packet_data += struct.pack('<B', checksum)           # Checksum
        packet_data += struct.pack('<H', self.PACKET_TAIL)   # Tail
        
        return packet_data
    
    def get_color_data(self, x, y, w, h):
        """
        获取颜色检测数据包 / Get color detection data packet
        
        Args:
            x, y, w, h: 检测框坐标和尺寸 / Detection box coordinates and size
            
        Returns:
            bytes: 数据包 / Data packet
        """
        # 数据格式: x(2) + y(2) + w(2) + h(2) + timestamp(4)
        data = struct.pack('<HHHHI', 
                          int(x) & 0xFFFF, 
                          int(y) & 0xFFFF, 
                          int(w) & 0xFFFF, 
                          int(h) & 0xFFFF,
                          int(time.ticks_ms()) & 0xFFFFFFFF)
        
        return self._pack_data(self.FUNC_COLOR_DETECT, data)
    
    def get_line_tracking_data(self, center_x, angle, found):
        """
        获取循迹数据包 / Get line tracking data packet
        
        Args:
            center_x: 线条中心X坐标 / Line center X coordinate
            angle: 线条角度 / Line angle
            found: 是否找到线条 / Whether line is found
            
        Returns:
            bytes: 数据包 / Data packet
        """
        # 数据格式: center_x(2) + angle(2, 有符号) + found(1) + timestamp(4)
        angle_int = int(angle * 100) & 0xFFFF  # 角度乘以100保留精度 / Angle * 100 for precision
        data = struct.pack('<HhBI', 
                          int(center_x) & 0xFFFF,
                          angle_int,
                          1 if found else 0,
                          int(time.ticks_ms()) & 0xFFFFFFFF)
        
        return self._pack_data(self.FUNC_LINE_TRACKING, data)
    
    def get_object_tracking_data(self, x, y, w, h, confidence):
        """
        获取目标追踪数据包 / Get object tracking data packet
        
        Args:
            x, y, w, h: 目标框坐标和尺寸 / Target box coordinates and size
            confidence: 置信度 / Confidence
            
        Returns:
            bytes: 数据包 / Data packet
        """
        # 数据格式: x(2) + y(2) + w(2) + h(2) + confidence(2) + timestamp(4)
        conf_int = int(confidence * 1000) & 0xFFFF  # 置信度乘以1000保留精度 / Confidence * 1000 for precision
        data = struct.pack('<HHHHI', 
                          int(x) & 0xFFFF, 
                          int(y) & 0xFFFF, 
                          int(w) & 0xFFFF, 
                          int(h) & 0xFFFF,
                          conf_int,
                          int(time.ticks_ms()) & 0xFFFFFFFF)
        
        return self._pack_data(self.FUNC_OBJECT_TRACKING, data)
    
    def get_nano_tracker_data(self, x, y, w, h):
        """
        获取纳米追踪器数据包 / Get nano tracker data packet
        
        Args:
            x, y, w, h: 追踪框坐标和尺寸 / Tracking box coordinates and size
            
        Returns:
            bytes: 数据包 / Data packet
        """
        # 数据格式: x(2) + y(2) + w(2) + h(2) + timestamp(4)
        data = struct.pack('<HHHHI', 
                          int(x) & 0xFFFF, 
                          int(y) & 0xFFFF, 
                          int(w) & 0xFFFF, 
                          int(h) & 0xFFFF,
                          int(time.ticks_ms()) & 0xFFFFFFFF)
        
        return self._pack_data(self.FUNC_NANO_TRACKER, data)
    
    def get_status_data(self, mode, line_color, tracking_status):
        """
        获取状态数据包 / Get status data packet
        
        Args:
            mode: 当前模式 / Current mode
            line_color: 循迹颜色索引 / Line tracking color index
            tracking_status: 追踪状态 / Tracking status
            
        Returns:
            bytes: 数据包 / Data packet
        """
        # 数据格式: mode(1) + line_color(1) + tracking_status(1) + timestamp(4)
        data = struct.pack('<BBBI', 
                          mode & 0xFF,
                          line_color & 0xFF,
                          tracking_status & 0xFF,
                          int(time.ticks_ms()) & 0xFFFFFFFF)
        
        return self._pack_data(self.FUNC_STATUS, data)

class SimpleUart:
    """
    简化的UART通信类 / Simplified UART communication class
    """
    
    def __init__(self, uart_id=1, baudrate=115200):
        """
        初始化UART / Initialize UART
        
        Args:
            uart_id: UART ID
            baudrate: 波特率 / Baud rate
        """
        try:
            self.uart = UART(uart_id, baudrate)
            self.connected = True
            print(f"UART{uart_id} 初始化成功，波特率: {baudrate} / UART{uart_id} initialized successfully, baudrate: {baudrate}")
        except Exception as e:
            print(f"UART初始化失败 / UART initialization failed: {e}")
            self.uart = None
            self.connected = False
    
    def send(self, data):
        """
        发送数据 / Send data
        
        Args:
            data: 要发送的数据 / Data to send
        """
        if self.uart and self.connected:
            try:
                if isinstance(data, str):
                    self.uart.write(data.encode())
                else:
                    self.uart.write(data)
                return True
            except Exception as e:
                print(f"数据发送失败 / Data send failed: {e}")
                return False
        return False
    
    def receive(self, timeout=100):
        """
        接收数据 / Receive data
        
        Args:
            timeout: 超时时间(ms) / Timeout (ms)
            
        Returns:
            bytes: 接收到的数据 / Received data
        """
        if self.uart and self.connected:
            try:
                if self.uart.any():
                    return self.uart.read()
            except Exception as e:
                print(f"数据接收失败 / Data receive failed: {e}")
        return None
    
    def any(self):
        """
        检查是否有数据可读 / Check if data is available
        
        Returns:
            bool: 是否有数据 / Whether data is available
        """
        if self.uart and self.connected:
            try:
                return self.uart.any() > 0
            except:
                return False
        return False
    
    def readline(self):
        """
        读取一行数据 / Read a line of data
        
        Returns:
            bytes: 一行数据 / One line of data
        """
        if self.uart and self.connected:
            try:
                return self.uart.readline()
            except Exception as e:
                print(f"读取行数据失败 / Read line failed: {e}")
        return b''

# 使用示例 / Usage example
if __name__ == "__main__":
    # 创建协议和UART实例 / Create protocol and UART instances
    protocol = SimpleProtocol()
    uart = SimpleUart(uart_id=1, baudrate=115200)
    
    # 测试数据包生成 / Test packet generation
    print("测试数据包生成 / Testing packet generation:")
    
    # 颜色检测数据包 / Color detection packet
    color_packet = protocol.get_color_data(100, 200, 50, 60)
    print(f"颜色检测数据包 / Color detection packet: {color_packet.hex()}")
    
    # 循迹数据包 / Line tracking packet
    line_packet = protocol.get_line_tracking_data(320, -15.5, True)
    print(f"循迹数据包 / Line tracking packet: {line_packet.hex()}")
    
    # 目标追踪数据包 / Object tracking packet
    track_packet = protocol.get_object_tracking_data(150, 180, 80, 90, 0.85)
    print(f"目标追踪数据包 / Object tracking packet: {track_packet.hex()}")
    
    # 状态数据包 / Status packet
    status_packet = protocol.get_status_data(2, 1, 1)
    print(f"状态数据包 / Status packet: {status_packet.hex()}")