import time
import math
import os
import gc
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()


def init_camera():
    """
    Initialize camera sensor with specified settings
    初始化摄像头传感器及其参数设置
    """
    sensor = Sensor()
    sensor.reset()  # Reset sensor to default settings | 重置传感器到默认设置
    sensor.set_framesize(width=640, height=480)  # Set resolution | 设置分辨率
    sensor.set_pixformat(Sensor.RGB565)  # Set pixel format | 设置像素格式
    return sensor

def init_display(sensor_obj):
    """
    Initialize display device
    初始化显示设备
    """
    Display.init(Display.ST7701, to_ide=True)  # Initialize ST7701 display | 初始化ST7701显示器
    # Alternative virtual display initialization | 备用虚拟显示器初始化
    #Display.init(Display.VIRT, sensor_obj.width(), sensor_obj.height())

def process_qrcode(image, qr_result):
    """
    Process detected QR code and draw information on image
    处理检测到的二维码并在图像上绘制信息

    Args:
        image: Current frame image | 当前帧图像
        qr_result: QR code detection result | 二维码检测结果
    """
    if len(qr_result) > 0:
        # Draw rectangle around QR code | 在二维码周围画矩形
        image.draw_rectangle(qr_result[0].rect(), thickness=2, color=(200, 0, 0))
        # Display QR code content | 显示二维码内容
        image.draw_string_advanced(0, 0, 30, qr_result[0].payload(),
                                 color=(255, 255, 255))
        print(qr_result[0].payload())

        x, y, w, h = qr_result[0].rect()
        pto_data = pto.get_qrcode_data(x, y, w, h, qr_result[0].payload())
        uart.send(pto_data)
        print(pto_data)


def main():
    """
    Main function to run QR code detection loop
    运行二维码检测的主函数
    """
    # Initialize system components | 初始化系统组件
    sensor = init_camera()
    init_display(sensor)
    MediaManager.init()
    sensor.run()

    # Initialize FPS clock | 初始化FPS计时器
    clock = time.clock()

    try:
        while True:
            clock.tick()  # Update FPS clock | 更新FPS计时器

            # Capture frame | 捕获图像帧
            img = sensor.snapshot()

            # Detect QR codes | 检测二维码
            qr_codes = img.find_qrcodes()

            # Process detection results | 处理检测结果
            process_qrcode(img, qr_codes)

            # Display result | 显示结果
            Display.show_image(img)
            print(clock.fps())  # Print current FPS | 打印当前FPS

    except KeyboardInterrupt:
        print("Program terminated by user")
        # Clean up resources | 清理资源
        sensor.close()
        gc.collect()

if __name__ == "__main__":
    main()
