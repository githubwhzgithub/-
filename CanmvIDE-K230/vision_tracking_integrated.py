#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
K230视觉循迹和目标追踪集成模块 (改进版)
K230 Vision Line Tracking and Object Tracking Integrated Module (Improved)

功能特性 / Features:
1. 颜色循迹 / Color Line Tracking
2. 目标追踪 / Object Tracking  
3. 自动模式切换 / Automatic Mode Switching
4. UART通信 / UART Communication
5. 基于PipeLine框架的规范化实现 / Standardized implementation based on PipeLine framework

作者 / Author: 眉峰藏雪
版本 / Version: 2.0 (基于K230文档优化 / Optimized based on K230 documentation)
日期 / Date: 2024
"""

# 核心库导入 / Core library imports
from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import time
import math
import gc
import sys
import os
import aidemo

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()

# 系统配置参数 / System configuration parameters
class Config:
    # 显示参数 / Display parameters
    DISPLAY_WIDTH = 640
    DISPLAY_HEIGHT = 480
    RGB888P_SIZE = [1280, 720]
    DISPLAY_SIZE = [1920, 1080]  # 根据实际显示设备调整 / Adjust according to actual display device
    DISPLAY_MODE = "lcd"  # "lcd" 或 "hdmi" / "lcd" or "hdmi"
    
    # 工作模式定义 / Working mode definitions
    MODE_IDLE = 0          # 空闲模式 / Idle mode
    MODE_LINE_TRACKING = 1 # 循迹模式 / Line tracking mode
    MODE_OBJECT_TRACKING = 2 # 目标追踪模式 / Object tracking mode
    MODE_AUTO_SWITCH = 3   # 自动切换模式 / Auto switch mode
    
    # 循迹颜色阈值 (LAB色彩空间) / Line tracking color thresholds (LAB color space)
    LINE_THRESHOLDS = [
        (0, 66, 7, 127, 3, 127),      # 红色线 / Red line
        (42, 100, -128, -17, 6, 66),  # 绿色线 / Green line
        (43, 99, -43, -4, -56, -7),   # 蓝色线 / Blue line
        (0, 30, -128, 127, -128, 127) # 黑色线 / Black line
    ]
    
    # 目标追踪参数 / Object tracking parameters
    TRACKING_CONFIDENCE = 0.5
    TRACKING_NMS_THRESHOLD = 0.2
    TRACKING_AREA_THRESHOLD = 2000
    
    # 性能参数 / Performance parameters
    DEBUG_MODE = 0  # 0: 计时模式, 1: 不计时 / 0: timing mode, 1: no timing
    MODE_SWITCH_THRESHOLD = 3000  # 模式切换阈值(ms) / Mode switch threshold (ms)
    
    # 通信参数 / Communication parameters
    UART_BAUDRATE = 115200
    SEND_INTERVAL = 50  # 数据发送间隔(ms) / Data send interval (ms)

class LineTrackingApp(AIBase):
    """
    循迹应用类 / Line tracking application class
    基于AIBase的规范化实现 / Standardized implementation based on AIBase
    """
    
    def __init__(self, rgb888p_size=None, display_size=None, debug_mode=0):
        """
        初始化循迹应用 / Initialize line tracking application
        """
        # 使用配置参数 / Use configuration parameters
        rgb888p_size = rgb888p_size or Config.RGB888P_SIZE
        display_size = display_size or Config.DISPLAY_SIZE
        
        # 调用父类初始化 / Call parent class initialization
        super().__init__(None, None, rgb888p_size, debug_mode)
        
        self.display_size = display_size
        self.line_color_index = 0
        self.line_center_x = 0
        self.line_angle = 0
        self.detection_confidence = 0.0
        
    def line_tracking(self, img):
        """
        执行循迹算法 / Execute line tracking algorithm
        
        Args:
            img: 输入图像 / Input image
            
        Returns:
            tuple: (center_x, angle, confidence, found) 线条信息
        """
        threshold = Config.LINE_THRESHOLDS[self.line_color_index]
        
        # 查找色块 / Find color blobs
        blobs = img.find_blobs([threshold], area_threshold=1000, merge=True)
        
        if blobs:
            # 找到最大的色块作为线条 / Find largest blob as line
            largest_blob = max(blobs, key=lambda b: b.area())
            
            # 计算线条中心和角度 / Calculate line center and angle
            center_x = largest_blob.cx()
            center_y = largest_blob.cy()
            
            # 使用线性回归计算角度 / Use linear regression to calculate angle
            try:
                line = img.get_regression([largest_blob], robust=True)
                if line:
                    angle = math.degrees(line.theta())
                    if angle > 90:
                        angle = angle - 180
                    # 计算置信度 / Calculate confidence
                    confidence = min(largest_blob.area() / 10000, 1.0) * line.magnitude()
                else:
                    angle = 0
                    confidence = min(largest_blob.area() / 10000, 1.0)
            except:
                angle = 0
                confidence = min(largest_blob.area() / 10000, 1.0)
                
            # 绘制检测结果 / Draw detection results
            img.draw_rectangle(largest_blob.rect(), color=(255, 0, 0), thickness=2)
            img.draw_cross(center_x, center_y, color=(0, 255, 0), thickness=2)
            
            # 绘制方向线 / Draw direction line
            if abs(angle) < 45:  # 只在角度合理时绘制 / Only draw when angle is reasonable
                line_length = 50
                end_x = int(center_x + line_length * math.cos(math.radians(angle)))
                end_y = int(center_y + line_length * math.sin(math.radians(angle)))
                img.draw_line(center_x, center_y, end_x, end_y, color=(0, 0, 255), thickness=2)
            
            self.line_center_x = center_x
            self.line_angle = angle
            self.detection_confidence = confidence
            
            return center_x, angle, confidence, True
        
        return Config.DISPLAY_WIDTH // 2, 0, 0.0, False
    
    def set_line_color(self, color_index):
        """
        设置循迹颜色 / Set line tracking color
        
        Args:
            color_index: 颜色索引 / Color index (0-3)
        """
        if 0 <= color_index < len(Config.LINE_THRESHOLDS):
            self.line_color_index = color_index
            print(f"循迹颜色设置为索引: {color_index} / Line color set to index: {color_index}")
    
    def run(self, input_np):
        """
        AIBase标准接口 / AIBase standard interface
        
        Args:
            input_np: 输入图像数组 / Input image array
            
        Returns:
            dict: 循迹结果 / Tracking result
        """
        # 将numpy数组转换为image对象 / Convert numpy array to image object
        img = image.Image(input_np.shape[1], input_np.shape[0], image.RGB565)
        img = img.from_bytes(input_np.tobytes())
        
        center_x, angle, confidence, found = self.line_tracking(img)
        
        return {
            'found': found,
            'center_x': center_x,
            'angle': angle,
            'confidence': confidence,
            'offset': center_x - Config.DISPLAY_WIDTH // 2
        }

class ObjectTrackingApp(AIBase):
    """
    目标追踪应用类 / Object tracking application class
    基于AIBase的规范化实现 / Standardized implementation based on AIBase
    """
    
    def __init__(self, rgb888p_size=None, display_size=None, debug_mode=0):
        """
        初始化目标追踪应用 / Initialize object tracking application
        """
        # 使用配置参数 / Use configuration parameters
        rgb888p_size = rgb888p_size or Config.RGB888P_SIZE
        display_size = display_size or Config.DISPLAY_SIZE
        
        # 调用父类初始化 / Call parent class initialization
        super().__init__(None, None, rgb888p_size, debug_mode)
        
        self.display_size = display_size
        self.tracking_box = None
        self.tracking_confidence = 0.0
        
    def object_tracking(self, img):
        """
        执行目标追踪算法 / Execute object tracking algorithm
        
        Args:
            img: 输入图像 / Input image
            
        Returns:
            tuple: (x, y, w, h, confidence, found) 目标位置和置信度
        """
        # 简化的目标检测：使用颜色检测作为目标追踪 / Simplified object detection using color detection
        # 在实际应用中，这里应该使用深度学习模型 / In practice, deep learning models should be used here
        
        # 使用多种颜色阈值检测目标 / Use multiple color thresholds to detect targets
        target_thresholds = [
            (30, 100, 15, 127, 15, 127),    # 红色目标 / Red target
            (30, 100, -64, -8, -32, 32),    # 绿色目标 / Green target
            (0, 30, 0, 64, -128, 0),        # 蓝色目标 / Blue target
        ]
        
        best_blob = None
        best_score = 0
        
        for threshold in target_thresholds:
            blobs = img.find_blobs([threshold], area_threshold=Config.TRACKING_AREA_THRESHOLD, merge=True)
            
            for blob in blobs:
                # 计算目标评分（面积 + 圆形度） / Calculate target score (area + circularity)
                area_score = min(blob.area() / 10000, 1.0)  # 归一化面积分数 / Normalized area score
                roundness_score = blob.roundness()  # 圆形度分数 / Roundness score
                total_score = area_score * 0.7 + roundness_score * 0.3
                
                if total_score > best_score:
                    best_score = total_score
                    best_blob = blob
        
        if best_blob and best_score > Config.TRACKING_CONFIDENCE:
            x, y, w, h = best_blob.rect()
            
            # 绘制追踪框 / Draw tracking box
            img.draw_rectangle(x, y, w, h, color=(255, 255, 0), thickness=3)
            img.draw_cross(best_blob.cx(), best_blob.cy(), color=(255, 0, 255), thickness=2)
            
            # 显示置信度 / Display confidence
            confidence_text = f"Conf: {best_score:.2f}"
            img.draw_string(x, y - 20, confidence_text, color=(255, 255, 255), scale=1)
            
            self.tracking_box = (x, y, w, h)
            self.tracking_confidence = best_score
            
            return x, y, w, h, best_score, True
        
        return 0, 0, 0, 0, 0.0, False
    


class VisionTrackingSystem:
    """
    视觉追踪系统主类 / Main vision tracking system class
    基于PipeLine框架的规范化实现 / Standardized implementation based on PipeLine framework
    """
    
    def __init__(self, rgb888p_size=None, display_size=None, display_mode=None, debug_mode=0):
        """
        初始化视觉追踪系统 / Initialize vision tracking system
        """
        # 使用配置参数 / Use configuration parameters
        self.rgb888p_size = rgb888p_size or Config.RGB888P_SIZE
        self.display_size = display_size or Config.DISPLAY_SIZE
        self.display_mode = display_mode or Config.DISPLAY_MODE
        self.debug_mode = debug_mode
        
        # 系统状态 / System state
        self.current_mode = Config.MODE_IDLE
        self.last_detection_time = 0
        self.last_send_time = 0
        
        # 初始化PipeLine / Initialize PipeLine
        self.pl = PipeLine(
            rgb888p_size=self.rgb888p_size,
            display_size=self.display_size,
            display_mode=self.display_mode,
            debug_mode=self.debug_mode
        )
        
        # 初始化应用模块 / Initialize application modules
        self.line_app = LineTrackingApp(self.rgb888p_size, self.display_size, debug_mode)
        self.track_app = ObjectTrackingApp(self.rgb888p_size, self.display_size, debug_mode)
        
        print(f"视觉追踪系统初始化完成 / Vision tracking system initialized")
        print(f"RGB888P尺寸: {self.rgb888p_size} / RGB888P size: {self.rgb888p_size}")
        print(f"显示尺寸: {self.display_size} / Display size: {self.display_size}")
        print(f"显示模式: {self.display_mode} / Display mode: {self.display_mode}")
        
    def set_mode(self, mode):
        """
        设置工作模式 / Set working mode
        
        Args:
            mode: 工作模式 / Working mode
        """
        if mode != self.current_mode:
            print(f"模式切换: {self.current_mode} -> {mode} / Mode switch: {self.current_mode} -> {mode}")
            self.current_mode = mode
            # 重置追踪状态 / Reset tracking state
            if hasattr(self, 'track_app'):
                self.track_app.tracking_box = None
                self.track_app.tracking_confidence = 0.0
            
    def set_line_color(self, color_index):
        """
        设置循迹颜色 / Set line tracking color
        
        Args:
            color_index: 颜色索引 / Color index (0-3)
        """
        if hasattr(self, 'line_app'):
            self.line_app.set_line_color(color_index)
            
    def line_tracking(self, img):
        """
        执行循迹算法 / Execute line tracking algorithm
        
        Args:
            img: 输入图像 / Input image
            
        Returns:
            dict: 循迹结果 / Tracking result
        """
        if hasattr(self, 'line_app'):
            center_x, angle, confidence, found = self.line_app.line_tracking(img)
            return {
                'found': found,
                'center_x': center_x,
                'angle': angle,
                'confidence': confidence,
                'offset': center_x - Config.DISPLAY_WIDTH // 2
            }
        else:
            return {
                'found': False,
                'center_x': Config.DISPLAY_WIDTH // 2,
                'angle': 0,
                'confidence': 0.0,
                'offset': 0
            }
    
    def object_tracking(self, img):
        """
        执行目标追踪算法 / Execute object tracking algorithm
        
        Args:
            img: 输入图像 / Input image
            
        Returns:
            dict: 追踪结果 / Tracking result
        """
        if hasattr(self, 'track_app'):
            x, y, w, h, confidence, found = self.track_app.object_tracking(img)
            return {
                'found': found,
                'center_x': x + w // 2 if found else Config.DISPLAY_WIDTH // 2,
                'center_y': y + h // 2 if found else Config.DISPLAY_HEIGHT // 2,
                'x': x,
                'y': y,
                'width': w,
                'height': h,
                'confidence': confidence
            }
        else:
            return {
                'found': False,
                'center_x': Config.DISPLAY_WIDTH // 2,
                'center_y': Config.DISPLAY_HEIGHT // 2,
                'x': 0,
                'y': 0,
                'width': 0,
                'height': 0,
                'confidence': 0.0
            }
    

    
    def auto_mode_switch(self, img):
        """
        自动模式切换逻辑 / Automatic mode switching logic
        
        Args:
            img: 输入图像 / Input image
        """
        current_time = time.ticks_ms()
        
        # 尝试检测线条 / Try to detect lines
        line_result = self.line_tracking(img)
        line_found = line_result['found']
        
        # 尝试检测目标 / Try to detect targets
        target_result = self.object_tracking(img)
        target_found = target_result['found']
        
        # 根据检测结果切换模式 / Switch mode based on detection results
        if line_found and not target_found:
            if self.current_mode != Config.MODE_LINE_TRACKING:
                if current_time - self.last_detection_time > Config.MODE_SWITCH_THRESHOLD:
                    self.set_mode(Config.MODE_LINE_TRACKING)
                    self.last_detection_time = current_time
        elif target_found and not line_found:
            if self.current_mode != Config.MODE_OBJECT_TRACKING:
                if current_time - self.last_detection_time > Config.MODE_SWITCH_THRESHOLD:
                    self.set_mode(Config.MODE_OBJECT_TRACKING)
                    self.last_detection_time = current_time
        elif line_found and target_found:
            # 两者都检测到时，优先目标追踪 / When both detected, prioritize object tracking
            if self.current_mode != Config.MODE_OBJECT_TRACKING:
                self.set_mode(Config.MODE_OBJECT_TRACKING)
                self.last_detection_time = current_time
    
    def send_data(self, mode, data):
        """
        发送数据到STM32 / Send data to STM32
        
        Args:
            mode: 当前模式 / Current mode
            data: 数据字典 / Data dictionary
        """
        try:
            current_time = time.ticks_ms()
            if current_time - self.last_send_time < Config.SEND_INTERVAL:
                return
                
            self.last_send_time = current_time
            
            if pto:  # 使用协议库 / Use protocol library
                if mode == Config.MODE_LINE_TRACKING:
                    # 发送循迹数据 / Send line tracking data
                    pto_data = pto.get_color_data(
                        data.get('center_x', Config.DISPLAY_WIDTH // 2),
                        Config.DISPLAY_HEIGHT // 2,
                        50,  # 虚拟宽度 / Virtual width
                        50   # 虚拟高度 / Virtual height
                    )
                elif mode == Config.MODE_OBJECT_TRACKING:
                    # 发送目标追踪数据 / Send object tracking data
                    pto_data = pto.get_nano_tracker_data(
                        data.get('x', 0),
                        data.get('y', 0),
                        data.get('width', 0),
                        data.get('height', 0)
                    )
                else:
                    return
                    
                uart.send(pto_data)
                if Config.DEBUG_MODE:
                    print(f"发送数据 / Sent data: {pto_data}")
            else:
                # 简单的数据格式 / Simple data format
                if mode == Config.MODE_LINE_TRACKING:
                    msg = f"LINE:{data.get('center_x', Config.DISPLAY_WIDTH // 2)},{data.get('angle', 0)}\n"
                elif mode == Config.MODE_OBJECT_TRACKING:
                    msg = f"TRACK:{data.get('x', 0)},{data.get('y', 0)},{data.get('width', 0)},{data.get('height', 0)}\n"
                else:
                    msg = "IDLE\n"
                    
                uart.write(msg.encode())
                if Config.DEBUG_MODE:
                    print(f"发送数据 / Sent data: {msg.strip()}")
                
        except Exception as e:
            if Config.DEBUG_MODE:
                print(f"数据发送失败 / Data send failed: {e}")
    
    def draw_ui(self, img, fps):
        """
        绘制用户界面 / Draw user interface
        
        Args:
            img: 图像对象 / Image object
            fps: 帧率 / Frame rate
        """
        try:
            # 绘制模式信息 / Draw mode information
            mode_names = ["IDLE", "LINE", "TRACK", "AUTO"]
            mode_text = f"Mode: {mode_names[self.current_mode]}"
            img.draw_string(10, 10, mode_text, color=(255, 255, 255), scale=2)
            
            # 绘制FPS / Draw FPS
            fps_text = f"FPS: {fps:.1f}"
            img.draw_string(10, 40, fps_text, color=(255, 255, 255), scale=1)
            
            # 绘制循迹信息 / Draw line tracking info
            if self.current_mode == Config.MODE_LINE_TRACKING or self.current_mode == Config.MODE_AUTO_SWITCH:
                if hasattr(self, 'line_app'):
                    color_names = ["Red", "Green", "Blue", "Black"]
                    color_text = f"Line: {color_names[self.line_app.line_color_index]}"
                    img.draw_string(10, 70, color_text, color=(255, 255, 255), scale=1)
                    
                    if hasattr(self.line_app, 'line_center_x'):
                        center_text = f"Center: {self.line_app.line_center_x}"
                        angle_text = f"Angle: {self.line_app.line_angle:.1f}"
                        img.draw_string(10, 100, center_text, color=(255, 255, 255), scale=1)
                        img.draw_string(10, 130, angle_text, color=(255, 255, 255), scale=1)
            
            # 绘制追踪信息 / Draw tracking info
            elif self.current_mode == Config.MODE_OBJECT_TRACKING or self.current_mode == Config.MODE_AUTO_SWITCH:
                if hasattr(self, 'track_app') and self.track_app.tracking_box:
                    x, y, w, h = self.track_app.tracking_box
                    track_text = f"Target: ({x},{y}) {w}x{h}"
                    conf_text = f"Conf: {self.track_app.tracking_confidence:.2f}"
                    img.draw_string(10, 70, track_text, color=(255, 255, 255), scale=1)
                    img.draw_string(10, 100, conf_text, color=(255, 255, 255), scale=1)
            
            # 绘制中心线 / Draw center line
            center_x = Config.DISPLAY_WIDTH // 2
            img.draw_line(center_x, 0, center_x, Config.DISPLAY_HEIGHT, color=(128, 128, 128), thickness=1)
            
        except Exception as e:
            if Config.DEBUG_MODE:
                print(f"UI绘制错误 / UI draw error: {e}")
    
    def process_frame(self):
        """
        处理单帧图像 / Process single frame
        
        Returns:
            bool: 是否继续运行 / Whether to continue running
        """
        try:
            img = self.pl.get_frame()
            
            # 根据当前模式处理图像 / Process image based on current mode
            if self.current_mode == Config.MODE_LINE_TRACKING:
                result = self.line_tracking(img)
                if result['found']:
                    self.send_data(Config.MODE_LINE_TRACKING, result)
                    
            elif self.current_mode == Config.MODE_OBJECT_TRACKING:
                result = self.object_tracking(img)
                if result['found']:
                    self.send_data(Config.MODE_OBJECT_TRACKING, result)
                    
            elif self.current_mode == Config.MODE_AUTO_SWITCH:
                self.auto_mode_switch(img)
            
            # 绘制UI / Draw UI
            fps = self.clock.fps()
            self.draw_ui(img, fps)
            
            # 显示图像 / Display image
            Display.show_image(img)
            
            # 垃圾回收 / Garbage collection
            gc.collect()
            
            return True
            
        except KeyboardInterrupt:
            print("用户中断 / User interrupted")
            return False
        except Exception as e:
            if Config.DEBUG_MODE:
                print(f"处理帧时出错 / Error processing frame: {e}")
            return True  # 继续运行 / Continue running
    
    def run(self):
        """
        主运行循环 / Main run loop
        """
        print("启动视觉追踪系统 / Starting vision tracking system...")
        
        try:
            # 启动传感器 / Start sensor
            self.sensor.run()
            
            # 设置默认模式 / Set default mode
            self.set_mode(Config.MODE_AUTO_SWITCH)
            
            # 主循环 / Main loop
            while True:
                if not self.process_frame():
                    break
                    
        except Exception as e:
            print(f"系统运行错误 / System runtime error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """
        清理资源 / Cleanup resources
        """
        print("清理资源 / Cleaning up resources...")
        try:
            if hasattr(self, 'sensor'):
                self.sensor.stop()
            MediaManager.deinit()
        except:
            pass

# 命令处理函数 / Command processing functions
def process_uart_command(vision_system):
    """
    处理UART命令 / Process UART commands
    
    Args:
        vision_system: 视觉系统实例 / Vision system instance
    """
    try:
        if uart.any():
            cmd = uart.readline().decode().strip()
            print(f"收到命令 / Received command: {cmd}")
            
            if cmd == "MODE_IDLE":
                vision_system.set_mode(Config.MODE_IDLE)
            elif cmd == "MODE_LINE":
                vision_system.set_mode(Config.MODE_LINE_TRACKING)
            elif cmd == "MODE_TRACK":
                vision_system.set_mode(Config.MODE_OBJECT_TRACKING)
            elif cmd == "MODE_AUTO":
                vision_system.set_mode(Config.MODE_AUTO_SWITCH)
            elif cmd.startswith("COLOR_"):
                color_index = int(cmd.split("_")[1])
                vision_system.set_line_color(color_index)
            elif cmd == "STATUS":
                status = {
                    'mode': vision_system.current_mode,
                    'line_color': vision_system.line_app.line_color_index if hasattr(vision_system, 'line_app') else 0
                }
                uart.write(f"STATUS:{status}\n".encode())
                
    except Exception as e:
        if Config.DEBUG_MODE:
            print(f"命令处理错误 / Command processing error: {e}")

def main():
    """
    主函数 / Main function
    """
    vision_system = None
    try:
        print("初始化K230视觉循迹和目标追踪系统... / Initializing K230 vision tracking system...")
        
        # 创建视觉系统实例 / Create vision system instance
        vision_system = VisionTrackingSystem()
        
        # 运行系统 / Run system
        vision_system.run()
        
    except Exception as e:
        print(f"系统错误 / System error: {e}")
    finally:
        # 清理资源 / Cleanup resources
        if vision_system:
            vision_system.cleanup()
        print("系统已退出 / System exited")

if __name__ == "__main__":
    main()