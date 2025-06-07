# Import required modules
# 导入所需模块
import time, math, os, gc

from media.sensor import *
from media.display import *
from media.media import *

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()


# AprilTag code supports processing up to 6 tag families simultaneously
# The returned tag object will have its tag family and ID within that family
# AprilTag代码最多支持同时处理6种tag家族
# 返回的tag标记对象包含其所属家族及在该家族中的ID

# Initialize tag families bitmask
# 初始化tag家族位掩码
tag_families = 0
tag_families |= image.TAG16H5    # Comment out to disable this family / 注释掉以禁用此家族
tag_families |= image.TAG25H7    # Comment out to disable this family / 注释掉以禁用此家族
tag_families |= image.TAG25H9    # Comment out to disable this family / 注释掉以禁用此家族
tag_families |= image.TAG36H10   # Comment out to disable this family / 注释掉以禁用此家族
tag_families |= image.TAG36H11   # Comment out to disable this family (default) / 注释掉以禁用此家族(默认)
tag_families |= image.ARTOOLKIT  # Comment out to disable this family / 注释掉以禁用此家族

def family_name(tag):
    """
    Get the family name of a tag
    获取tag的家族名称

    Args:
        tag: AprilTag object / AprilTag对象
    Returns:
        str: Name of the tag family / tag家族名称
    """
    family_dict = {
        image.TAG16H5: "TAG16H5",
        image.TAG25H7: "TAG25H7",
        image.TAG25H9: "TAG25H9",
        image.TAG36H10: "TAG36H10",
        image.TAG36H11: "TAG36H11",
        image.ARTOOLKIT: "ARTOOLKIT"
    }
    return family_dict.get(tag.family())

# Initialize camera sensor
# 初始化摄像头传感器
sensor = Sensor()
sensor.reset()
sensor.set_framesize(width=400, height=240)
sensor.set_pixformat(Sensor.RGB565)

# Initialize display
# 初始化显示
Display.init(Display.ST7701, width=640, height=480, to_ide=True)
#Display.init(Display.VIRT, sensor.width(), sensor.height())

# Initialize media manager and start sensor
# 初始化媒体管理器并启动传感器
MediaManager.init()
sensor.run()

# Create clock for FPS calculation
# 创建时钟用于FPS计算
clock = time.clock()

# Main loop
# 主循环
while True:
    clock.tick()

    # Capture image
    # 捕获图像
    img = sensor.snapshot()

    # Find and process AprilTags
    # 查找和处理AprilTags
    for tag in img.find_apriltags(families=tag_families):
        # Draw rectangle and cross on detected tag
        # 在检测到的tag上绘制矩形和十字
        img.draw_rectangle(tag.rect(), color=(255, 0, 0), thickness=4)
        img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0), thickness=2)

        # Print tag information
        # 打印tag信息
        print_args = (family_name(tag), tag.id(), (180 * tag.rotation()) / math.pi)
        print("Tag Family %s, Tag ID %d, rotation %f (degrees)" % print_args)

        x, y, w, h = tag.rect()
        pto_data = pto.get_apriltag_data(x, y, w, h, tag.id(), (180*tag.rotation())/math.pi)
        uart.send(pto_data)
        print(pto_data)
        break

    # Display image centered on screen
    # 在屏幕中央显示图像
    Display.show_image(img, x=120, y=120)

    # Print frames per second
    # 打印帧率
    print(clock.fps())
