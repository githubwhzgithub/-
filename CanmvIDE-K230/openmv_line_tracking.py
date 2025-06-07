# OpenMV循迹追踪算法 / OpenMV Line Tracking Algorithm
# 基于MicroPython实现 / Based on MicroPython implementation
# 版本: 1.0 / Version: 1.0
# 作者: AI Assistant / Author: AI Assistant
# 日期: 2024 / Date: 2024

import media.sensor
import image
from time import *
import math
from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart

# 配置参数 / Configuration Parameters
class Config:
    # 显示参数 / Display Parameters
    DISPLAY_WIDTH = 320
    DISPLAY_HEIGHT = 240

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
    MIN_BLOB_AREA = 100      # 最小色块面积 / Minimum blob area
    MAX_BLOB_AREA = 10000    # 最大色块面积 / Maximum blob area
    ROI_Y_START = 120        # 感兴趣区域起始Y坐标 / ROI start Y coordinate
    ROI_HEIGHT = 120         # 感兴趣区域高度 / ROI height

    # 通信参数 / Communication Parameters
    UART_BAUDRATE = 115200   # 串口波特率 / UART baud rate
    SEND_INTERVAL = 50       # 发送间隔(ms) / Send interval (ms)

    # 调试模式 / Debug Mode
    DEBUG_MODE = True

class PIDController:
    """
    PID控制器 / PID Controller
    """
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        """
        更新PID控制器 / Update PID controller

        Args:
            error: 误差值 / Error value
            dt: 时间间隔 / Time interval

        Returns:
            float: 控制输出 / Control output
        """
        # 比例项 / Proportional term
        proportional = self.kp * error

        # 积分项 / Integral term
        self.integral += error * dt
        integral = self.ki * self.integral

        # 微分项 / Derivative term
        derivative = self.kd * (error - self.previous_error) / dt if dt > 0 else 0

        # 更新前一次误差 / Update previous error
        self.previous_error = error

        # 返回控制输出 / Return control output
        return proportional + integral + derivative

    def reset(self):
        """
        重置PID控制器 / Reset PID controller
        """
        self.previous_error = 0
        self.integral = 0

class LineTracker:
    """
    循迹追踪器 / Line Tracker
    """
    def __init__(self):
        # 初始化传感器 / Initialize sensor
        self.init_sensor()

        # 初始化PID控制器 / Initialize PID controller
        self.pid = PIDController(Config.PID_KP, Config.PID_KI, Config.PID_KD)

        # 初始化串口通信 / Initialize UART communication
        self.uart = UART(3, Config.UART_BAUDRATE)

        # 状态变量 / State variables
        self.current_color = 'black'  # 当前追踪颜色 / Current tracking color
        self.line_found = False       # 是否找到线条 / Line found flag
        self.line_center_x = Config.DISPLAY_WIDTH // 2  # 线条中心X坐标 / Line center X
        self.line_angle = 0           # 线条角度 / Line angle
        self.confidence = 0.0         # 检测置信度 / Detection confidence

        # 时间记录 / Time tracking
        self.last_time = time.ticks_ms()
        self.last_send_time = 0

        # LED指示 / LED indicators
        self.red_led = LED(1)
        self.green_led = LED(2)
        self.blue_led = LED(3)

    def init_sensor(self):
        """
        初始化摄像头传感器 / Initialize camera sensor
        """
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)  # 320x240
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)

        if Config.DEBUG_MODE:
            print("传感器初始化完成 / Sensor initialized")

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
                print(f"设置追踪颜色为: {color_name} / Set tracking color to: {color_name}")

    def find_line(self, img):
        """
        查找线条 / Find line

        Args:
            img: 输入图像 / Input image

        Returns:
            tuple: (center_x, angle, confidence, found) 线条信息
        """
        # 定义感兴趣区域 / Define region of interest
        roi = (0, Config.ROI_Y_START, Config.DISPLAY_WIDTH, Config.ROI_HEIGHT)

        # 获取当前颜色阈值 / Get current color threshold
        threshold = Config.COLOR_THRESHOLDS[self.current_color]

        # 查找色块 / Find blobs
        blobs = img.find_blobs([threshold],
                              roi=roi,
                              pixels_threshold=Config.MIN_BLOB_AREA,
                              area_threshold=Config.MIN_BLOB_AREA,
                              merge=True)

        if blobs:
            # 找到最大的色块 / Find the largest blob
            largest_blob = max(blobs, key=lambda b: b.area())

            # 过滤面积过大的色块 / Filter blobs that are too large
            if largest_blob.area() > Config.MAX_BLOB_AREA:
                return Config.DISPLAY_WIDTH // 2, 0, 0.0, False

            # 计算线条中心 / Calculate line center
            center_x = largest_blob.cx()
            center_y = largest_blob.cy()

            # 计算线条角度 / Calculate line angle
            angle = largest_blob.rotation_deg()

            # 计算置信度 / Calculate confidence
            confidence = min(largest_blob.area() / Config.MAX_BLOB_AREA, 1.0)

            # 绘制检测结果 / Draw detection results
            img.draw_rectangle(largest_blob.rect(), color=(255, 0, 0))
            img.draw_cross(center_x, center_y, color=(0, 255, 0), size=10)

            # 绘制方向线 / Draw direction line
            line_length = 30
            end_x = int(center_x + line_length * math.cos(math.radians(angle)))
            end_y = int(center_y + line_length * math.sin(math.radians(angle)))
            img.draw_line(center_x, center_y, end_x, end_y, color=(0, 0, 255), thickness=2)

            return center_x, angle, confidence, True

        return Config.DISPLAY_WIDTH // 2, 0, 0.0, False

    def calculate_control(self, center_x):
        """
        计算控制输出 / Calculate control output

        Args:
            center_x: 线条中心X坐标 / Line center X coordinate

        Returns:
            float: 控制输出 / Control output
        """
        # 计算误差 / Calculate error
        target_x = Config.DISPLAY_WIDTH // 2
        error = center_x - target_x

        # 计算时间间隔 / Calculate time interval
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.last_time) / 1000.0
        self.last_time = current_time

        # 使用PID控制器计算输出 / Use PID controller to calculate output
        control_output = self.pid.update(error, dt)

        return control_output

    def send_data(self, center_x, angle, control_output, found):
        """
        发送数据到STM32 / Send data to STM32

        Args:
            center_x: 线条中心X坐标 / Line center X coordinate
            angle: 线条角度 / Line angle
            control_output: 控制输出 / Control output
            found: 是否找到线条 / Line found flag
        """
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, self.last_send_time) < Config.SEND_INTERVAL:
            return

        self.last_send_time = current_time

        try:
            # 构造数据包 / Construct data packet
            data = f"LINE:{int(center_x)},{int(angle)},{int(control_output*100)},{int(found)}\n"
            self.uart.write(data.encode())

            if Config.DEBUG_MODE:
                print(f"发送数据: {data.strip()} / Sent data: {data.strip()}")

        except Exception as e:
            if Config.DEBUG_MODE:
                print(f"数据发送失败: {e} / Data send failed: {e}")

    def update_leds(self, found, control_output):
        """
        更新LED指示 / Update LED indicators

        Args:
            found: 是否找到线条 / Line found flag
            control_output: 控制输出 / Control output
        """
        if found:
            self.green_led.on()  # 绿灯表示找到线条 / Green LED indicates line found
            self.red_led.off()

            # 蓝灯表示转向方向 / Blue LED indicates turning direction
            if abs(control_output) > 0.1:
                self.blue_led.on()
            else:
                self.blue_led.off()
        else:
            self.red_led.on()    # 红灯表示未找到线条 / Red LED indicates line not found
            self.green_led.off()
            self.blue_led.off()

    def draw_ui(self, img, fps):
        """
        绘制用户界面 / Draw user interface

        Args:
            img: 图像对象 / Image object
            fps: 帧率 / Frame rate
        """
        # 绘制基本信息 / Draw basic information
        img.draw_string(10, 10, f"Color: {self.current_color}", color=(255, 255, 255), scale=1)
        img.draw_string(10, 30, f"FPS: {fps:.1f}", color=(255, 255, 255), scale=1)
        img.draw_string(10, 50, f"Found: {self.line_found}", color=(255, 255, 255), scale=1)

        if self.line_found:
            img.draw_string(10, 70, f"Center: {self.line_center_x}", color=(255, 255, 255), scale=1)
            img.draw_string(10, 90, f"Angle: {self.line_angle:.1f}", color=(255, 255, 255), scale=1)
            img.draw_string(10, 110, f"Conf: {self.confidence:.2f}", color=(255, 255, 255), scale=1)

        # 绘制中心线 / Draw center line
        center_x = Config.DISPLAY_WIDTH // 2
        img.draw_line(center_x, 0, center_x, Config.DISPLAY_HEIGHT, color=(128, 128, 128), thickness=1)

        # 绘制ROI区域 / Draw ROI area
        img.draw_rectangle(0, Config.ROI_Y_START, Config.DISPLAY_WIDTH, Config.ROI_HEIGHT,
                          color=(255, 255, 0), thickness=1)

    def process_uart_commands(self):
        """
        处理串口命令 / Process UART commands
        """
        try:
            if self.uart.any():
                cmd = self.uart.readline().decode().strip()
                if Config.DEBUG_MODE:
                    print(f"收到命令: {cmd} / Received command: {cmd}")

                if cmd.startswith("COLOR_"):
                    color_name = cmd.split("_")[1].lower()
                    if color_name in Config.COLOR_THRESHOLDS:
                        self.set_tracking_color(color_name)
                elif cmd == "RESET":
                    self.pid.reset()
                elif cmd == "STATUS":
                    status = f"STATUS:{self.current_color},{self.line_found},{self.line_center_x}\n"
                    self.uart.write(status.encode())

        except Exception as e:
            if Config.DEBUG_MODE:
                print(f"命令处理错误: {e} / Command processing error: {e}")

    def run(self):
        """
        主运行循环 / Main run loop
        """
        print("启动OpenMV循迹追踪系统... / Starting OpenMV line tracking system...")

        # 初始化时钟 / Initialize clock
        clock = time.clock()

        try:
            while True:
                clock.tick()

                # 捕获图像 / Capture image
                img = sensor.snapshot()

                # 处理串口命令 / Process UART commands
                self.process_uart_commands()

                # 查找线条 / Find line
                center_x, angle, confidence, found = self.find_line(img)

                # 更新状态 / Update state
                self.line_found = found
                self.line_center_x = center_x
                self.line_angle = angle
                self.confidence = confidence

                # 计算控制输出 / Calculate control output
                control_output = 0
                if found:
                    control_output = self.calculate_control(center_x)
                else:
                    self.pid.reset()  # 重置PID控制器 / Reset PID controller

                # 发送数据 / Send data
                self.send_data(center_x, angle, control_output, found)

                # 更新LED指示 / Update LED indicators
                self.update_leds(found, control_output)

                # 绘制用户界面 / Draw user interface
                fps = clock.fps()
                self.draw_ui(img, fps)

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
            # 关闭LED / Turn off LEDs
            self.red_led.off()
            self.green_led.off()
            self.blue_led.off()

            # 关闭串口 / Close UART
            if hasattr(self, 'uart'):
                self.uart.deinit()

        except:
            pass

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
