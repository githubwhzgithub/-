# 导入必要的模块 Import required modules
import time
import gc
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager
import image

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()



# 定义条形码类型映射字典 Define barcode type mapping dictionary
BARCODE_TYPES = {
    image.EAN2: "EAN2",
    image.EAN5: "EAN5",
    image.EAN8: "EAN8",
    image.UPCE: "UPCE",
    image.ISBN10: "ISBN10",
    image.UPCA: "UPCA",
    image.EAN13: "EAN13",
    image.ISBN13: "ISBN13",
    image.I25: "I25",
    image.DATABAR: "DATABAR",
    image.DATABAR_EXP: "DATABAR_EXP",
    image.CODABAR: "CODABAR",
    image.CODE39: "CODE39",
    image.PDF417: "PDF417",
    image.CODE93: "CODE93",
    image.CODE128: "CODE128"
}

def barcode_name(code):
    """
    获取条形码类型名称
    Get barcode type name
    """
    return BARCODE_TYPES.get(code.type(), "UNKNOWN")

def init_camera():
    """
    初始化摄像头设置
    Initialize camera settings
    """
    sensor = Sensor()  # 构建摄像头对象 Create camera object
    sensor.reset()     # 复位和初始化摄像头 Reset and initialize camera
    # 设置帧大小为LCD分辨率(640x480) Set frame size to LCD resolution (640x480)
    sensor.set_framesize(width=640, height=480)
    # 设置输出图像格式 Set output image format
    sensor.set_pixformat(Sensor.RGB565)
    return sensor

def main():
    # 初始化摄像头 Initialize camera
    sensor = init_camera()

    # 初始化显示设置 Initialize display settings
    Display.init(Display.ST7701,width = 640, height = 480, to_ide=True)

    # 初始化media资源管理器 Initialize media resource manager
    MediaManager.init()

    # 启动sensor Start the sensor
    sensor.run()

    # 创建时钟对象用于FPS计算 Create clock object for FPS calculation
    clock = time.clock()

    while True:
        clock.tick()

        # 捕获图像 Capture image
        img = sensor.snapshot()

        # 查找图像中所有条形码 Find all barcodes in the image
        codes = img.find_barcodes()

        for code in codes:
            # 用矩形框标记条码位置 Mark barcode position with rectangle
            img.draw_rectangle(code.rect(), thickness=6, color=(46, 47, 48))

            # 获取条码类型和内容 Get barcode type and content
            code_type = barcode_name(code)
            payload = code.payload()

            # 打印条码信息 Print barcode information
            print(f"Barcode {code_type}, Payload \"{payload}\"")

            # 在图像中显示条码内容 Display barcode content in image
            img.draw_string_advanced(10, 10, 40, payload, color=(255, 255, 255))

            x, y, w, h = code.rect()
            pto_data = pto.get_barcode_data(x, y, w, h, payload)
            uart.send(pto_data)
            print(pto_data)
            break

        # 显示处理后的图像 Display processed image
        Display.show_image(img)

        # 执行垃圾回收 Perform garbage collection
        gc.collect()

if __name__ == "__main__":
    main()
