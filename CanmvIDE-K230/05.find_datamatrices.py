import time
import math
import os
import gc
import sys

from media.sensor import *
from media.display import *
from media.media import *

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()


# 定义检测图像的宽度和高度
# Define the width and height of the image for detection
DETECT_WIDTH = 640
DETECT_HEIGHT = 480

sensor = None

try:
    # 使用默认配置构造 Sensor 对象
    # Create a Sensor object with default configuration
    sensor = Sensor()
    # 重置 sensor
    # Reset the sensor
    sensor.reset()

    # 设置输出大小和格式
    # Set the output size and pixel format
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
#    sensor.set_pixformat(Sensor.GRAYSCALE)


    # 初始化显示
    # Initialize the display
    Display.init(Display.ST7701, width=DETECT_WIDTH, height=DETECT_HEIGHT, to_ide=True)

    # 初始化媒体管理器
    # Initialize the media manager
    MediaManager.init()

    # 启动 sensor
    # Start the sensor
    sensor.run()

    fps = time.clock()

    while True:
        fps.tick()

        # 检查是否应该退出
        # Check if we should exit
        if os.exitpoint():
            break

        img = sensor.snapshot()

        # 遍历图像中的 Data Matrix 条形码
        # Iterate through the Data Matrix codes found in the image
        for matrix in img.find_datamatrices():
            # 绘制识别到的 Data Matrix 码的矩形框
            # Draw the rectangle around the detected Data Matrix code
            (x, y, w, h) = matrix.rect()
            y = y - 25 if y - 25 > 0 else y
            img.draw_rectangle([v for v in matrix.rect()], color=(255, 0, 0), thickness=4)
            # 打印矩阵的行列数、内容、旋转角度（度）以及当前的 FPS
            # Print the matrix's row, column count, payload, rotation in degrees, and current FPS
            print_args = (matrix.payload(), (180 * matrix.rotation()) / math.pi)
            img.draw_string_advanced(x, y, 20, "%s [%.2f]°" % print_args, color=(255,0,0))

            print("payload \"%s\", rotation %f" % print_args)
            x, y, w, h = matrix.rect()
            pto_data = pto.get_dmcode_data(x, y, w, h, matrix.payload(), (180*matrix.rotation())/math.pi)
            uart.send(pto_data)
            print(pto_data)
            break

        # 将结果显示到屏幕上
        # Display the result on the screen
        Display.show_image(img)

        # 进行垃圾回收，释放内存
        # Perform garbage collection to release memory
        gc.collect()

except KeyboardInterrupt:
    print("用户停止")
    # User stopped
except BaseException as e:
    print(f"异常 '{e}'")
    # Exception caught
finally:
    # 停止 sensor
    # Stop the sensor
    if isinstance(sensor, Sensor):
        sensor.stop()

    # 销毁 display
    # Deinitialize the display
    Display.deinit()

    # 启用休眠模式
    # Enable sleep mode
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)

    # 释放媒体缓冲区
    # Release the media buffers
    MediaManager.deinit()
