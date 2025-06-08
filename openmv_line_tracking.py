# 循迹追踪算法 / OpenMV Line Tracking Algorithm
# 基于MicroPython实现 / Based on MicroPython implementation
# 版本: 2.0 / Version: 2.0
# 作者: 眉峰藏雪
# 日期: 2024 / Date: 2024

import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
import math

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart

# 配置参数 / Configuration Parameters
class Config:
    # 显示参数 / Display Parameters
    DISPLAY_WIDTH = 640
    DISPLAY_HEIGHT = 480

    # 循迹颜色阈值 / Line tracking color thresholds
    # 格式: (L_min, L_max, A_min, A_max, B_min, B_max)
    COLOR_THRESHOLDS = {
        'red': (30, 100, 15, 127, 15, 127),      # 红色 / Red
        'green': (30, 100, -64, -8, -32, 32),   # 绿色 / Green
        'blue': (0, 30, 0, 64, -128, 0),        # 蓝色 / Blue
        'black': (0, 30, -20, 20, -20, 20),     # 黑色 / Black
        'white': (80, 100, -20, 20, -20, 20)    # 白色 / White
    }

    # PID控制参数 / PID Control Parameters
    PID_KP = 0.8  # 比例系数 / Proportional coefficient
    PID_KI = 0.1  # 积分系数 / Integral coefficient
    PID_KD = 0.2  # 微分系数 / Derivative coefficient

    # 检测参数 / Detection Parameters
    MIN_BLOB_AREA = 100       # 最小色块面积 / Minimum blob area
    MAX_BLOB_AREA = 8000     # 最大色块面积 / Maximum blob area
    MIN_BLOB_PIXELS = 30     # 最小色块像素数 / Minimum blob pixels

    # 多ROI区域配置 / Multi-ROI configuration
    # 格式: (x, y, w, h, weight) - 权重越大表示越重要
    ROI_REGIONS = [
        (0, 400, 640, 30, 0.7),   # 下方区域 - 最重要 / Bottom region - most important
        (0, 350, 640, 30, 0.3),   # 中下区域 / Middle-bottom region
        (0, 300, 640, 30, 0.1),   # 中间区域 / Middle region
        (0, 250, 640, 30, 0.05),  # 中上区域 / Middle-top region
        (0, 200, 640, 30, 0.02)   # 上方区域 / Top region
    ]

    # 权重和计算 / Weight sum calculation
    WEIGHT_SUM = sum(roi[4] for roi in ROI_REGIONS)

    # 角度计算参数 / Angle calculation parameters
    ANGLE_THRESHOLD = 30     # 角度阈值 / Angle threshold
    DISTORTION_FACTOR = 1.5  # 畸变矫正因子 / Distortion correction factor

    # 通信参数 / Communication Parameters
    UART_BAUDRATE = 115200   # 串口波特率 / UART baud rate
    SEND_INTERVAL = 50       # 发送间隔(ms) / Send interval (ms)

    # 调试模式 / Debug Mode
    DEBUG_MODE = True

class PIDController:
    """
    增强型PID控制器 - 支持自适应参数和抗积分饱和 / Enhanced PID Controller with adaptive parameters and anti-windup
    """
    def __init__(self, kp, ki, kd, output_limit=100, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit or output_limit * 0.8  # 积分限幅

        self.last_error = 0
        self.integral = 0
        self.last_time = time.ticks_ms()
        self.error_history = []  # 误差历史用于自适应调整
        self.max_history_length = 10

        # 自适应参数
        self.base_kp = kp
        self.base_ki = ki
        self.base_kd = kd

    def update(self, error, confidence=1.0):
        """
        更新PID控制器 - 支持置信度调整 / Update PID controller with confidence adjustment

        Args:
            error: 误差值 / Error value
            confidence: 检测置信度 (0-1) / Detection confidence (0-1)

        Returns:
            float: 控制输出 / Control output
        """
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.last_time) / 1000.0

        if dt <= 0:
            return 0

        # 更新误差历史 / Update error history
        self.error_history.append(abs(error))
        if len(self.error_history) > self.max_history_length:
            self.error_history.pop(0)

        # 自适应参数调整 / Adaptive parameter adjustment
        self._adaptive_tuning(error, confidence)

        # 比例项 / Proportional term
        proportional = self.kp * error

        # 积分项 - 带抗饱和 / Integral term with anti-windup
        self.integral += error * dt
        # 积分限幅 / Integral clamping
        self.integral = max(-self.integral_limit/self.ki if self.ki != 0 else 0,
                           min(self.integral_limit/self.ki if self.ki != 0 else 0, self.integral))
        integral = self.ki * self.integral

        # 微分项 - 带滤波 / Derivative term with filtering
        derivative_raw = (error - self.last_error) / dt
        # 简单低通滤波 / Simple low-pass filter
        if hasattr(self, 'last_derivative'):
            derivative_filtered = 0.7 * self.last_derivative + 0.3 * derivative_raw
        else:
            derivative_filtered = derivative_raw
        self.last_derivative = derivative_filtered
        derivative = self.kd * derivative_filtered

        # 计算输出 / Calculate output
        output = proportional + integral + derivative

        # 输出限幅 / Output clamping
        clamped_output = max(-self.output_limit, min(self.output_limit, output))

        # 抗积分饱和 - 如果输出饱和，减少积分项 / Anti-windup - reduce integral if output saturated
        if abs(output) > self.output_limit and abs(self.integral) > 0:
            if (output > 0 and error > 0) or (output < 0 and error < 0):
                self.integral *= 0.9  # 减少积分项

        # 更新历史值 / Update history values
        self.last_error = error
        self.last_time = current_time

        return clamped_output

    def _adaptive_tuning(self, error, confidence):
        """
        自适应参数调整 / Adaptive parameter tuning
        """
        if len(self.error_history) < 3:
            return

        # 计算误差变化趋势 / Calculate error trend
        recent_errors = self.error_history[-3:]
        error_trend = sum(recent_errors) / len(recent_errors)

        # 根据置信度调整参数 / Adjust parameters based on confidence
        confidence_factor = max(0.5, confidence)  # 最小置信度因子

        # 根据误差大小和趋势调整Kp / Adjust Kp based on error magnitude and trend
        if abs(error) > 50:  # 大误差时增加Kp
            self.kp = self.base_kp * 1.2 * confidence_factor
        elif abs(error) < 10:  # 小误差时减少Kp避免震荡
            self.kp = self.base_kp * 0.8 * confidence_factor
        else:
            self.kp = self.base_kp * confidence_factor

        # 根据误差稳定性调整Ki / Adjust Ki based on error stability
        error_variance = sum([(e - error_trend)**2 for e in recent_errors]) / len(recent_errors)
        if error_variance < 5:  # 误差稳定时增加Ki
            self.ki = self.base_ki * 1.1 * confidence_factor
        else:  # 误差不稳定时减少Ki
            self.ki = self.base_ki * 0.9 * confidence_factor

        # 根据误差变化率调整Kd / Adjust Kd based on error change rate
        if len(self.error_history) >= 2:
            error_change_rate = abs(self.error_history[-1] - self.error_history[-2])
            if error_change_rate > 20:  # 变化率大时增加Kd
                self.kd = self.base_kd * 1.3 * confidence_factor
            else:
                self.kd = self.base_kd * confidence_factor

    def reset(self):
        """
        重置PID控制器 / Reset PID controller
        """
        self.last_error = 0
        self.integral = 0
        self.last_time = time.ticks_ms()
        self.error_history = []
        if hasattr(self, 'last_derivative'):
            delattr(self, 'last_derivative')

        # 恢复基础参数 / Restore base parameters
        self.kp = self.base_kp
        self.ki = self.base_ki
        self.kd = self.base_kd

class LineTracker:
    """
    循迹追踪器 / Line Tracker
    """
    def __init__(self):
        # 初始化传感器 / Initialize sensor
        self.sensor = self.init_sensor()

        # 初始化显示 / Initialize display
        self.init_display()

        # 初始化增强型PID控制器 / Initialize enhanced PID controller
        self.pid = PIDController(
            kp=Config.PID_KP,
            ki=Config.PID_KI,
            kd=Config.PID_KD,
            output_limit=100,
            integral_limit=80
        )

        # 初始化串口通信 / Initialize UART communication
        self.uart = YbUart(baudrate=Config.UART_BAUDRATE)
        self.pto = YbProtocol()

        # 状态变量 / State variables
        self.current_color = 'black'  # 当前追踪颜色 / Current tracking color
        self.line_found = False       # 是否找到线条 / Line found flag
        self.line_center_x = Config.DISPLAY_WIDTH // 2  # 线条中心X坐标 / Line center X
        self.line_angle = 0           # 线条角度 / Line angle
        self.confidence = 0.0         # 检测置信度 / Detection confidence

        # 时间记录 / Time tracking
        self.last_time = time.ticks_ms()
        self.last_send_time = 0

    def init_sensor(self):
        """
        初始化摄像头传感器 / Initialize camera sensor
        """
        sensor = Sensor()
        sensor.reset()
        sensor.set_framesize(width=Config.DISPLAY_WIDTH, height=Config.DISPLAY_HEIGHT)
        sensor.set_pixformat(Sensor.RGB565)

        if Config.DEBUG_MODE:
            print("传感器初始化完成 ")

        return sensor

    def init_display(self):
        """
        初始化显示 / Initialize display
        """
        Display.init(Display.ST7701, to_ide=True)
        MediaManager.init()

    def set_tracking_color(self, color_name):
        """
        设置追踪颜色 / Set tracking color

        Args:
            color_name: 颜色名称 / Color name
        """
        if color_name in Config.COLOR_THRESHOLDS:
            self.current_color = color_name
            self.pid.reset()  # 重置PID控制器 / Reset PID controller
            if Config.DEBUG_MODE:
                print(f"设置追踪颜色为: {color_name} ")

    def find_line(self, img):
        """
        查找线条 - 使用多ROI区域加权算法 / Find line using multi-ROI weighted algorithm

        Args:
            img: 输入图像 / Input image

        Returns:
            tuple: (center_x, angle, confidence, found) 线条信息
        """
        # 应用畸变矫正 / Apply distortion correction
        img = img.lens_corr(strength=Config.DISTORTION_FACTOR, zoom=1.0)

        # 获取当前颜色阈值 / Get current color threshold
        threshold = Config.COLOR_THRESHOLDS[self.current_color]

        # 存储每个ROI区域的检测结果 / Store detection results for each ROI
        roi_results = []
        weighted_center_sum = 0
        total_weight = 0
        valid_detections = 0

        # 遍历所有ROI区域 / Iterate through all ROI regions
        for i, (x, y, w, h, weight) in enumerate(Config.ROI_REGIONS):
            roi = (x, y, w, h)

            # 在当前ROI区域查找色块 / Find blobs in current ROI
            blobs = img.find_blobs([threshold],
                                  roi=roi,
                                  pixels_threshold=Config.MIN_BLOB_PIXELS,
                                  area_threshold=Config.MIN_BLOB_AREA,
                                  merge=True)

            # 绘制ROI区域 / Draw ROI region
            if Config.DEBUG_MODE:
                color = (255, 255, 0) if i == 0 else (128, 128, 128)  # 最重要的ROI用黄色
                img.draw_rectangle(roi, color=color, thickness=1)

            if blobs:
                # 找到最大的有效色块 / Find the largest valid blob
                valid_blobs = [blob for blob in blobs
                              if Config.MIN_BLOB_AREA <= blob.area() <= Config.MAX_BLOB_AREA
                              and blob.pixels() >= Config.MIN_BLOB_PIXELS
                              and blob.density() > 0.3]  # 密度过滤

                if valid_blobs:
                    largest_blob = max(valid_blobs, key=lambda b: b.area())
                    center_x = largest_blob.cx()

                    # 计算加权中心 / Calculate weighted center
                    weighted_center_sum += center_x * weight
                    total_weight += weight
                    valid_detections += 1

                    # 存储检测结果 / Store detection result
                    roi_results.append({
                        'roi_index': i,
                        'blob': largest_blob,
                        'center_x': center_x,
                        'weight': weight,
                        'area': largest_blob.area()
                    })

                    # 绘制检测到的色块 / Draw detected blob
                    if Config.DEBUG_MODE:
                        img.draw_rectangle(largest_blob.rect(), color=(255, 0, 0))
                        img.draw_cross(center_x, largest_blob.cy(), color=(0, 255, 0), size=5)

        # 如果没有检测到有效线条 / If no valid line detected
        if valid_detections == 0 or total_weight == 0:
            return Config.DISPLAY_WIDTH // 2, 0, 0.0, False

        # 计算加权平均中心位置 / Calculate weighted average center position
        weighted_center_x = weighted_center_sum / total_weight

        # 计算线条角度 - 使用最重要ROI区域的角度 / Calculate line angle using most important ROI
        line_angle = 0
        if roi_results:
            # 优先使用权重最大的检测结果 / Prioritize detection with highest weight
            primary_result = max(roi_results, key=lambda r: r['weight'])
            line_angle = primary_result['blob'].rotation_deg()

            # 如果有多个检测结果，进行角度平滑 / Smooth angle if multiple detections
            if len(roi_results) > 1:
                angle_sum = 0
                angle_weight_sum = 0
                for result in roi_results:
                    angle = result['blob'].rotation_deg()
                    weight = result['weight']
                    angle_sum += angle * weight
                    angle_weight_sum += weight
                line_angle = angle_sum / angle_weight_sum if angle_weight_sum > 0 else line_angle

        # 计算置信度 / Calculate confidence
        confidence = min(total_weight / Config.WEIGHT_SUM, 1.0)

        # 绘制最终结果 / Draw final result
        if Config.DEBUG_MODE:
            # 绘制加权中心线 / Draw weighted center line
            img.draw_line(int(weighted_center_x), 0, int(weighted_center_x), Config.DISPLAY_HEIGHT,
                         color=(0, 255, 255), thickness=2)

            # 绘制角度指示线 / Draw angle indicator line
            if abs(line_angle) > 5:  # 只有角度足够大时才绘制
                line_length = 50
                center_y = Config.DISPLAY_HEIGHT // 2
                end_x = int(weighted_center_x + line_length * math.cos(math.radians(line_angle)))
                end_y = int(center_y + line_length * math.sin(math.radians(line_angle)))
                img.draw_line(int(weighted_center_x), center_y, end_x, end_y,
                             color=(255, 0, 255), thickness=3)

        return int(weighted_center_x), line_angle, confidence, True

    def calculate_control(self, center_x, confidence=1.0, line_angle=0):
        """
        计算控制输出 - 支持动态速度调整 / Calculate control output with dynamic speed adjustment

        Args:
            center_x: 线条中心X坐标 / Line center X coordinate
            confidence: 检测置信度 / Detection confidence
            line_angle: 线条角度 / Line angle

        Returns:
            tuple: (control_output, speed_factor) 控制输出和速度因子
        """
        # 计算位置误差 / Calculate position error
        position_error = center_x - Config.DISPLAY_WIDTH // 2

        # 角度误差补偿 / Angle error compensation
        angle_compensation = 0
        if abs(line_angle) > Config.ANGLE_THRESHOLD:
            # 角度越大，需要更强的转向补偿
            angle_compensation = line_angle * 0.5

        # 综合误差 / Combined error
        total_error = position_error + angle_compensation

        # 使用增强型PID控制器计算输出 / Calculate output using enhanced PID controller
        control_output = self.pid.update(total_error, confidence)

        # 动态速度调整 / Dynamic speed adjustment
        speed_factor = self._calculate_speed_factor(abs(total_error), confidence, abs(line_angle))

        return control_output, speed_factor

    def _calculate_speed_factor(self, error_magnitude, confidence, angle_magnitude):
        """
        计算动态速度因子 / Calculate dynamic speed factor

        Args:
            error_magnitude: 误差幅度 / Error magnitude
            confidence: 检测置信度 / Detection confidence
            angle_magnitude: 角度幅度 / Angle magnitude

        Returns:
            float: 速度因子 (0.3-1.0) / Speed factor (0.3-1.0)
        """
        # 基础速度因子 / Base speed factor
        base_speed = 1.0

        # 根据误差大小调整速度 / Adjust speed based on error magnitude
        if error_magnitude > 80:
            error_factor = 0.4  # 大误差时慢速
        elif error_magnitude > 40:
            error_factor = 0.6  # 中等误差时中速
        elif error_magnitude > 20:
            error_factor = 0.8  # 小误差时较快
        else:
            error_factor = 1.0  # 很小误差时全速

        # 根据置信度调整速度 / Adjust speed based on confidence
        confidence_factor = max(0.5, confidence)  # 置信度低时减速

        # 根据角度调整速度 / Adjust speed based on angle
        if angle_magnitude > 30:
            angle_factor = 0.5  # 大角度时减速
        elif angle_magnitude > 15:
            angle_factor = 0.7  # 中等角度时中速
        else:
            angle_factor = 1.0  # 小角度时正常速度

        # 综合速度因子 / Combined speed factor
        speed_factor = base_speed * error_factor * confidence_factor * angle_factor

        # 限制速度因子范围 / Limit speed factor range
        return max(0.3, min(1.0, speed_factor))

    def send_data(self, center_x, angle, control_output, found, speed_factor=1.0):
        """
        发送数据到STM32 - 增强版本支持速度因子 / Send data to STM32 - Enhanced version with speed factor

        Args:
            center_x: 线条中心X坐标 / Line center X coordinate
            angle: 线条角度 / Line angle
            control_output: 控制输出 / Control output
            found: 是否找到线条 / Line found flag
            speed_factor: 速度因子 / Speed factor
        """
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, self.last_send_time) < Config.SEND_INTERVAL:
            return

        self.last_send_time = current_time

        try:
            # 使用YbProtocol构造数据包 / Construct data packet using YbProtocol
            # 使用颜色检测协议发送循迹数据，将角度和控制输出作为宽高参数
            # 将速度因子转换为百分比 / Convert speed factor to percentage
            speed_percentage = int(speed_factor * 100)
            pto_data = self.pto.get_color_data(int(center_x), int(angle), speed_percentage, int(found))
            self.uart.send(pto_data)

            if Config.DEBUG_MODE:
                print(f"发送数据: center_x={center_x}, angle={angle:.1f}, speed={speed_percentage}%, found={found}")

        except Exception as e:
            if Config.DEBUG_MODE:
                print(f"数据发送失败: {e} ")

    def update_status_display(self, found, control_output):
        """
        更新状态显示 / Update status display

        Args:
            found: 是否找到线条 / Line found flag
            control_output: 控制输出 / Control output
        """
        # 可以在这里添加状态指示逻辑
        # Status indication logic can be added here
        pass

    def draw_ui(self, img, fps, speed_factor=1.0):
        """
        绘制用户界面 - 增强版本 / Draw user interface - Enhanced version

        Args:
            img: 图像对象 / Image object
            fps: 帧率 / Frame rate
            speed_factor: 速度因子 / Speed factor
        """
        # 绘制基本信息 / Draw basic information
        img.draw_string_advanced(10, 10, 20, f"Color: {self.current_color}", color=(255, 255, 255))
        img.draw_string_advanced(10, 35, 20, f"FPS: {fps:.1f}", color=(255, 255, 255))
        img.draw_string_advanced(10, 60, 20, f"Found: {self.line_found}", color=(255, 255, 255))
        img.draw_string_advanced(10, 85, 20, f"Speed: {speed_factor:.1f}", color=(255, 255, 255))

        if self.line_found:
            img.draw_string_advanced(10, 110, 20, f"Center: {self.line_center_x}", color=(255, 255, 255))
            img.draw_string_advanced(10, 135, 20, f"Angle: {self.line_angle:.1f}", color=(255, 255, 255))
            img.draw_string_advanced(10, 160, 20, f"Conf: {self.confidence:.2f}", color=(255, 255, 255))

            # 绘制PID参数信息 / Draw PID parameters info
            if Config.DEBUG_MODE:
                img.draw_string_advanced(10, 185, 20, f"Kp: {self.pid.kp:.2f}", color=(128, 255, 128))
                img.draw_string_advanced(10, 210, 20, f"Ki: {self.pid.ki:.2f}", color=(128, 255, 128))
                img.draw_string_advanced(10, 235, 20, f"Kd: {self.pid.kd:.2f}", color=(128, 255, 128))

        # 绘制中心线 / Draw center line
        center_x = Config.DISPLAY_WIDTH // 2
        img.draw_line(center_x, 0, center_x, Config.DISPLAY_HEIGHT, color=(128, 128, 128), thickness=1)

        # 绘制速度指示条 / Draw speed indicator bar
        bar_width = int(200 * speed_factor)
        bar_color = (0, 255, 0) if speed_factor > 0.7 else (255, 255, 0) if speed_factor > 0.4 else (255, 0, 0)
        img.draw_rectangle(Config.DISPLAY_WIDTH - 220, 10, bar_width, 20, color=bar_color, fill=True)
        img.draw_rectangle(Config.DISPLAY_WIDTH - 220, 10, 200, 20, color=(255, 255, 255), thickness=2)

    def process_uart_commands(self):
        """
        处理串口命令 / Process UART commands
        """
        try:
            # 处理串口接收的命令 / Process UART received commands
            received_data = self.uart.read()
            if received_data:
                cmd = received_data.decode().strip()
                if Config.DEBUG_MODE:
                    print(f"收到命令: {cmd} ")

                if cmd.startswith("COLOR_"):
                    color_name = cmd.split("_")[1].lower()
                    if color_name in Config.COLOR_THRESHOLDS:
                        self.set_tracking_color(color_name)
                elif cmd == "RESET":
                    self.pid.reset()
                elif cmd == "STATUS":
                    # 发送状态信息，使用消息协议
                    status_msg = f"{self.current_color},{self.line_found},{self.line_center_x}"
                    status_data = self.pto.package_message(self.pto.ID_COLOR, status_msg)
                    self.uart.send(status_data)

        except Exception as e:
            if Config.DEBUG_MODE:
                print(f"命令处理错误: {e} ")

    def run(self):
        """
        主运行循环 / Main run loop
        """
        print("启动OpenMV循迹追踪系统... / Starting OpenMV line tracking system...")

        # 启动传感器 / Start sensor
        self.sensor.run()

        # 初始化时钟 / Initialize clock
        clock = time.clock()

        try:
            while True:
                clock.tick()

                # 捕获图像 / Capture image
                img = self.sensor.snapshot()

                # 处理串口命令 / Process UART commands
                self.process_uart_commands()

                # 查找线条 / Find line
                center_x, angle, confidence, found = self.find_line(img)

                # 更新状态 / Update state
                self.line_found = found
                self.line_center_x = center_x
                self.line_angle = angle
                self.confidence = confidence

                # 计算控制输出和速度因子 / Calculate control output and speed factor
                control_output = 0
                speed_factor = 1.0
                if found:
                    control_output, speed_factor = self.calculate_control(center_x, confidence, angle)
                else:
                    self.pid.reset()  # 重置PID控制器 / Reset PID controller
                    speed_factor = 0.3  # 未找到线条时低速前进

                # 发送数据 / Send data
                self.send_data(center_x, angle, control_output, found, speed_factor)

                # 更新状态显示 / Update status display
                self.update_status_display(found, control_output)

                # 绘制用户界面 / Draw user interface
                fps = clock.fps()
                self.draw_ui(img, fps, speed_factor)

                # 显示图像 / Display image
                Display.show_image(img)

                if Config.DEBUG_MODE and fps < 10:
                    print(f"警告: FPS过低 {fps:.1f} / Warning: Low FPS {fps:.1f}")

        except KeyboardInterrupt:
            print("用户中断 / User interrupted")
        except Exception as e:
            print(f"系统错误: {e} / System error: {e}")
        finally:
            # 清理资源 / Cleanup resources
            self.cleanup()

    def cleanup(self):
        """
        清理资源 / Cleanup resources
        """
        print("清理资源... / Cleaning up resources...")
        try:
            # 停止传感器 / Stop sensor
            if hasattr(self, 'sensor') and isinstance(self.sensor, Sensor):
                self.sensor.stop()

            # 关闭显示 / Close display
            Display.deinit()
            MediaManager.deinit()

            # 关闭串口 / Close UART
            if hasattr(self, 'uart'):
                self.uart.close()

        except Exception as e:
            if Config.DEBUG_MODE:
                print(f"清理资源时出错: {e} / Error during cleanup: {e}")

def main():
    """
    主函数 / Main function
    """
    tracker = None
    try:
        print("初始化OpenMV循迹追踪系统... / Initializing OpenMV line tracking system...")

        # 创建追踪器实例 / Create tracker instance
        tracker = LineTracker()

        # 设置默认追踪颜色 / Set default tracking color
        tracker.set_tracking_color('black')

        # 运行系统 / Run system
        tracker.run()

    except Exception as e:
        print(f"系统错误: {e} / System error: {e}")
    finally:
        # 清理资源 / Cleanup resources
        if tracker:
            tracker.cleanup()
        print("系统已退出 / System exited")

if __name__ == "__main__":
    main()
