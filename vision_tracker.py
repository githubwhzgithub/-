# 视觉追踪系统 / Vision Tracking System
# 融合循迹追踪和物体检测功能 / Integrated line tracking and object detection
# 版本: 1.0 / Version: 1.0
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

    # 工作模式 / Working modes
    MODE_LINE_TRACKING = 1    # 循迹模式 / Line tracking mode
    MODE_OBJECT_TRACKING = 2  # 物体追踪模式 / Object tracking mode

    # 循迹颜色阈值 / Line tracking color thresholds
    # 格式: (L_min, L_max, A_min, A_max, B_min, B_max)
    LINE_COLOR_THRESHOLDS = {
        'red': (30, 100, 15, 127, 15, 127),      # 红色 / Red
        'green': (30, 100, -64, -8, -32, 32),   # 绿色 / Green
        'blue': (0, 30, 0, 64, -128, 0),        # 蓝色 / Blue
        'black': (0, 30, -20, 20, -20, 20),     # 黑色 / Black
        'white': (80, 100, -20, 20, -20, 20)    # 白色 / White
    }

    # 物体检测颜色阈值 / Object detection color thresholds
    # (L Min, L Max, A Min, A Max, B Min, B Max)
    OBJECT_COLOR_THRESHOLDS = [
        (0, 66, 7, 127, 3, 127),              # 红色阈值 / Red threshold
        (42, 100, -128, -17, 6, 66),          # 绿色阈值 / Green threshold
        (43, 99, -43, -4, -56, -7),           # 蓝色阈值 / Blue threshold
    ]

    # PID控制参数 / PID Control Parameters
    PID_KP = 0.8  # 比例系数 / Proportional coefficient
    PID_KI = 0.1  # 积分系数 / Integral coefficient
    PID_KD = 0.2  # 微分系数 / Derivative coefficient

    # 检测参数 / Detection Parameters
    MIN_BLOB_AREA = 100       # 最小色块面积 / Minimum blob area
    MAX_BLOB_AREA = 8000     # 最大色块面积 / Maximum blob area
    MIN_BLOB_PIXELS = 30     # 最小色块像素数 / Minimum blob pixels
    OBJECT_MIN_AREA = 5000   # 物体检测最小面积 / Object detection minimum area

    # 多ROI区域配置（循迹模式） / Multi-ROI configuration (line tracking mode)
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

class VisionTracker:
    """
    视觉追踪器 - 支持循迹和物体检测两种模式 / Vision Tracker - supports both line tracking and object detection
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

        # 工作模式 / Working mode
        self.current_mode = Config.MODE_LINE_TRACKING  # 默认循迹模式 / Default line tracking mode

        # 循迹模式状态变量 / Line tracking mode state variables
        self.current_line_color = 'black'  # 当前追踪颜色 / Current tracking color
        self.line_found = False            # 是否找到线条 / Line found flag
        self.line_center_x = Config.DISPLAY_WIDTH // 2  # 线条中心X坐标 / Line center X
        self.line_angle = 0                # 线条角度 / Line angle
        self.confidence = 0.0              # 检测置信度 / Detection confidence

        # 物体检测模式状态变量 / Object detection mode state variables
        self.current_object_color_index = 0  # 当前检测的颜色索引 / Current detection color index
        self.object_found = False            # 是否找到物体 / Object found flag
        self.object_center_x = Config.DISPLAY_WIDTH // 2   # 物体中心X坐标 / Object center X
        self.object_center_y = Config.DISPLAY_HEIGHT // 2  # 物体中心Y坐标 / Object center Y
        self.object_width = 0                # 物体宽度 / Object width
        self.object_height = 0               # 物体高度 / Object height

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
            print("传感器初始化完成 / Sensor initialized")

        return sensor

    def init_display(self):
        """
        初始化显示 / Initialize display
        """
        Display.init(Display.ST7701, to_ide=True)
        MediaManager.init()

    def set_mode(self, mode):
        """
        设置工作模式 / Set working mode

        Args:
            mode: 工作模式 (1: 循迹, 2: 物体检测) / Working mode (1: line tracking, 2: object detection)
        """
        if mode in [Config.MODE_LINE_TRACKING, Config.MODE_OBJECT_TRACKING]:
            self.current_mode = mode
            self.pid.reset()  # 重置PID控制器 / Reset PID controller
            if Config.DEBUG_MODE:
                mode_name = "循迹模式" if mode == Config.MODE_LINE_TRACKING else "物体检测模式"
                print(f"设置工作模式为: {mode_name} ")

    def set_line_tracking_color(self, color_name):
        """
        设置循迹颜色 / Set line tracking color

        Args:
            color_name: 颜色名称 / Color name
        """
        if color_name in Config.LINE_COLOR_THRESHOLDS:
            self.current_line_color = color_name
            self.pid.reset()  # 重置PID控制器 / Reset PID controller
            if Config.DEBUG_MODE:
                print(f"设置循迹颜色为: {color_name} ")

    def set_object_detection_color(self, color_index):
        """
        设置物体检测颜色 / Set object detection color

        Args:
            color_index: 颜色索引 (0:红, 1:绿, 2:蓝) 
        """
        if 0 <= color_index < len(Config.OBJECT_COLOR_THRESHOLDS):
            self.current_object_color_index = color_index
            self.pid.reset()  # 重置PID控制器 / Reset PID controller
            if Config.DEBUG_MODE:
                color_names = ["红色", "绿色", "蓝色"]
                print(f"设置物体检测颜色为: {color_names[color_index]}")

    def get_closest_rgb(self, lab_threshold):
        """
        根据LAB阈值计算最接近的RGB颜色 
        """
        # 获取LAB空间的中心点值
        l_center = (lab_threshold[0] + lab_threshold[1]) // 2
        a_center = (lab_threshold[2] + lab_threshold[3]) // 2
        b_center = (lab_threshold[4] + lab_threshold[5]) // 2
        return image.lab_to_rgb((l_center, a_center, b_center))

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
        threshold = Config.LINE_COLOR_THRESHOLDS[self.current_line_color]

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

    def find_object(self, img):
        """
        查找物体 - 物体检测模式 / Find object - object detection mode

        Args:
            img: 输入图像 / Input image

        Returns:
            tuple: (center_x, center_y, width, height, found) 物体信息
        """
        # 获取当前颜色阈值 / Get current color threshold
        threshold = Config.OBJECT_COLOR_THRESHOLDS[self.current_object_color_index]
        detect_color = self.get_closest_rgb(threshold)

        # 检测指定颜色 / Detect specified color
        blobs = img.find_blobs([threshold], area_threshold=Config.OBJECT_MIN_AREA, merge=True)

        if blobs:
            # 找到最大的色块 / Find the largest blob
            largest_blob = max(blobs, key=lambda b: b.area())

            # 绘制检测到的色块 / Draw detected blob
            img.draw_rectangle(largest_blob.rect(), color=detect_color, thickness=4)
            img.draw_cross(largest_blob.cx(), largest_blob.cy(), color=detect_color, thickness=2)

            return largest_blob.cx(), largest_blob.cy(), largest_blob.w(), largest_blob.h(), True

        return Config.DISPLAY_WIDTH // 2, Config.DISPLAY_HEIGHT // 2, 0, 0, False

    def calculate_control(self, center_x, confidence=1.0, line_angle=0):
        """
        计算控制输出 - 支持动态速度调整 / Calculate control output with dynamic speed adjustment

        Args:
            center_x: 目标中心X坐标 / Target center X coordinate
            confidence: 检测置信度 / Detection confidence
            line_angle: 线条角度（仅循迹模式使用） / Line angle (only used in line tracking mode)

        Returns:
            tuple: (control_output, speed_factor) 控制输出和速度因子
        """
        # 计算位置误差 / Calculate position error
        position_error = center_x - Config.DISPLAY_WIDTH // 2

        # 角度误差补偿（仅循迹模式） / Angle error compensation (only for line tracking mode)
        angle_compensation = 0
        if self.current_mode == Config.MODE_LINE_TRACKING and abs(line_angle) > Config.ANGLE_THRESHOLD:
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

        # 根据角度调整速度（仅循迹模式） / Adjust speed based on angle (only for line tracking mode)
        angle_factor = 1.0
        if self.current_mode == Config.MODE_LINE_TRACKING:
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

    def send_data(self, center_x, param2, param3, found, speed_factor=1.0):
        """
        发送数据到STM32 / Send data to STM32

        Args:
            center_x: 目标中心X坐标 / Target center X coordinate
            param2: 第二个参数（循迹模式：角度，物体检测模式：中心Y坐标） / Second parameter
            param3: 第三个参数（循迹模式：速度百分比，物体检测模式：宽度） / Third parameter
            found: 是否找到目标 / Target found flag
            speed_factor: 速度因子 / Speed factor
        """
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, self.last_send_time) < Config.SEND_INTERVAL:
            return

        self.last_send_time = current_time

        try:
            # 根据STM32端K230_ParseData函数的协议要求构造数据包
            if self.current_mode == Config.MODE_LINE_TRACKING:
                # 循迹模式数据格式: $length,1,center_x,angle,speed_percentage,found#
                # 功能ID=1 对应STM32端的K230_MODE_LINE_TRACK
                speed_percentage = int(speed_factor * 100)
                func_id = 1  # K230_MODE_LINE_TRACK
                pto_data = self.pto.package_coord(func_id, int(center_x), int(param2), speed_percentage, int(found))
            else:
                # 物体检测模式数据格式: $length,2,center_x,center_y,width,found#
                # 功能ID=2 对应STM32端的K230_MODE_OBJECT_TRACK
                func_id = 2  # K230_MODE_OBJECT_TRACK
                pto_data = self.pto.package_coord(func_id, int(center_x), int(param2), int(param3), int(found))

            self.uart.send(pto_data)

            if Config.DEBUG_MODE:
                if self.current_mode == Config.MODE_LINE_TRACKING:
                    print(f"循迹数据: center_x={center_x}, angle={param2:.1f}, speed={param3}%, found={found}")
                else:
                    print(f"物体数据: center_x={center_x}, center_y={param2}, width={param3}, found={found}")

        except Exception as e:
            if Config.DEBUG_MODE:
                print(f"数据发送失败: {e} / Data send failed: {e}")

    def draw_ui(self, img, fps, speed_factor=1.0):
        """
        绘制用户界面 / Draw user interface

        Args:
            img: 图像对象 / Image object
            fps: 帧率 / Frame rate
            speed_factor: 速度因子 / Speed factor
        """
        # 绘制基本信息 / Draw basic information
        mode_name = "循迹" if self.current_mode == Config.MODE_LINE_TRACKING else "物体检测"
        img.draw_string_advanced(10, 10, 20, f"Mode: {mode_name}", color=(255, 255, 255))
        img.draw_string_advanced(10, 35, 20, f"FPS: {fps:.1f}", color=(255, 255, 255))

        if self.current_mode == Config.MODE_LINE_TRACKING:
            # 循迹模式信息 / Line tracking mode info
            img.draw_string_advanced(10, 60, 20, f"Color: {self.current_line_color}", color=(255, 255, 255))
            img.draw_string_advanced(10, 85, 20, f"Found: {self.line_found}", color=(255, 255, 255))
            img.draw_string_advanced(10, 110, 20, f"Speed: {speed_factor:.1f}", color=(255, 255, 255))

            if self.line_found:
                img.draw_string_advanced(10, 135, 20, f"Center: {self.line_center_x}", color=(255, 255, 255))
                img.draw_string_advanced(10, 160, 20, f"Angle: {self.line_angle:.1f}", color=(255, 255, 255))
                img.draw_string_advanced(10, 185, 20, f"Conf: {self.confidence:.2f}", color=(255, 255, 255))
        else:
            # 物体检测模式信息 / Object detection mode info
            color_names = ["Red", "Green", "Blue", "YAHBOOM"]
            img.draw_string_advanced(10, 60, 20, f"Color: {color_names[self.current_object_color_index]}", color=(255, 255, 255))
            img.draw_string_advanced(10, 85, 20, f"Found: {self.object_found}", color=(255, 255, 255))

            if self.object_found:
                img.draw_string_advanced(10, 110, 20, f"X: {self.object_center_x}", color=(255, 255, 255))
                img.draw_string_advanced(10, 135, 20, f"Y: {self.object_center_y}", color=(255, 255, 255))
                img.draw_string_advanced(10, 160, 20, f"Size: {self.object_width}x{self.object_height}", color=(255, 255, 255))

        # 绘制中心线 / Draw center line
        center_x = Config.DISPLAY_WIDTH // 2
        img.draw_line(center_x, 0, center_x, Config.DISPLAY_HEIGHT, color=(128, 128, 128), thickness=1)

        # 绘制速度指示条（仅循迹模式） / Draw speed indicator bar (only for line tracking mode)
        if self.current_mode == Config.MODE_LINE_TRACKING:
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
                    print(f"收到命令: {cmd} / Received command: {cmd}")

                if cmd.startswith("MODE_"):
                    # 设置工作模式 / Set working mode
                    mode_str = cmd.split("_")[1]
                    if mode_str == "1" or mode_str.upper() == "LINE":
                        self.set_mode(Config.MODE_LINE_TRACKING)
                    elif mode_str == "2" or mode_str.upper() == "OBJECT":
                        self.set_mode(Config.MODE_OBJECT_TRACKING)
                elif cmd.startswith("COLOR_"):
                    # 设置颜色 / Set color
                    if self.current_mode == Config.MODE_LINE_TRACKING:
                        color_name = cmd.split("_")[1].lower()
                        if color_name in Config.LINE_COLOR_THRESHOLDS:
                            self.set_line_tracking_color(color_name)
                    else:
                        color_index_str = cmd.split("_")[1]
                        try:
                            color_index = int(color_index_str)
                            self.set_object_detection_color(color_index)
                        except ValueError:
                            pass
                elif cmd == "RESET":
                    self.pid.reset()
                elif cmd == "STATUS":
                    # 发送状态信息 / Send status information
                    if self.current_mode == Config.MODE_LINE_TRACKING:
                        status_msg = f"{self.current_mode},{self.current_line_color},{self.line_found},{self.line_center_x}"
                    else:
                        status_msg = f"{self.current_mode},{self.current_object_color_index},{self.object_found},{self.object_center_x}"
                    status_data = self.pto.package_message(self.pto.ID_COLOR, status_msg)
                    self.uart.send(status_data)

        except Exception as e:
            if Config.DEBUG_MODE:
                print(f"命令处理错误: {e} / Command processing error: {e}")

    def run(self):
        """
        主运行循环 / Main run loop
        """
        print("启动视觉追踪系统... / Starting vision tracking system...")

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

                # 根据当前模式进行处理 / Process according to current mode
                if self.current_mode == Config.MODE_LINE_TRACKING:
                    # 循迹模式 / Line tracking mode
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
                    speed_percentage = int(speed_factor * 100)
                    self.send_data(center_x, angle, speed_percentage, found, speed_factor)

                else:
                    # 物体检测模式 / Object detection mode
                    center_x, center_y, width, height, found = self.find_object(img)

                    # 更新状态 / Update state
                    self.object_found = found
                    self.object_center_x = center_x
                    self.object_center_y = center_y
                    self.object_width = width
                    self.object_height = height

                    # 计算控制输出 / Calculate control output
                    control_output = 0
                    speed_factor = 1.0
                    if found:
                        control_output, speed_factor = self.calculate_control(center_x, 1.0, 0)
                    else:
                        self.pid.reset()  # 重置PID控制器 / Reset PID controller

                    # 发送数据 / Send data
                    self.send_data(center_x, center_y, width, found)

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
        print("初始化视觉追踪系统... / Initializing vision tracking system...")

        # 创建追踪器实例 / Create tracker instance
        tracker = VisionTracker()

        # 设置默认模式和参数 / Set default mode and parameters
        tracker.set_mode(Config.MODE_OBJECT_TRACKING)  # 默认物体追踪模式
        tracker.set_line_tracking_color('black')     # 默认追踪黑色线条
        tracker.set_object_detection_color(0)        # 默认检测红色物体

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