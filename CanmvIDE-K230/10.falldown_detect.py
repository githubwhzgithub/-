from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os
import ujson
from media.media import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import time
import utime
import image
import random
import gc
import sys
import aicube
from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()

# 自定义跌倒检测类，继承自AIBase基类
# Custom fall detection class that inherits from AIBase base class
class FallDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, labels, anchors, confidence_threshold=0.2, nms_threshold=0.5, nms_option=False, strides=[8,16,32], rgb888p_size=[224,224], display_size=[1920,1080], debug_mode=0):
        # 调用基类的构造函数
        # Call the constructor of the parent class
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)

        # 模型文件路径
        # Path to the model file
        self.kmodel_path = kmodel_path

        # 模型输入分辨率
        # Model input resolution
        self.model_input_size = model_input_size

        # 分类标签
        # Classification labels
        self.labels = labels

        # 锚点数据，用于跌倒检测
        # Anchor data for fall detection
        self.anchors = anchors

        # 步长设置，用于特征图的尺度变换
        # Stride settings for feature map scale transformation
        self.strides = strides

        # 置信度阈值，检测结果的最小置信度要求
        # Confidence threshold, minimum confidence requirement for detection results
        self.confidence_threshold = confidence_threshold

        # NMS（非极大值抑制）阈值，用于过滤重叠框
        # NMS (Non-Maximum Suppression) threshold for filtering overlapping boxes
        self.nms_threshold = nms_threshold

        # NMS选项
        # NMS option
        self.nms_option = nms_option

        # sensor给到AI的图像分辨率，并对宽度进行16的对齐
        # Image resolution from sensor to AI, with width aligned to multiples of 16
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]

        # 显示分辨率，并对宽度进行16的对齐
        # Display resolution, with width aligned to multiples of 16
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]

        # 是否开启调试模式
        # Whether to enable debug mode
        self.debug_mode = debug_mode

        # 用于绘制不同类别的颜色 (RGBA格式)
        # Colors for drawing different categories (RGBA format)
        self.color = [(255,0, 0, 255), (255,0, 255, 0), (255,255,0, 0), (255,255,0, 255)]

        # Ai2d实例，用于实现模型预处理
        # Ai2d instance for model preprocessing
        self.ai2d = Ai2d(debug_mode)

        # 设置Ai2d的输入输出格式和类型
        # Set input and output format and type for Ai2d
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    # 配置预处理操作，这里使用了pad和resize
    # Configure preprocessing operations, using pad and resize here
    # Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/app/libs/AI2D.py查看
    # Ai2d supports crop/shift/pad/resize/affine, check /sdcard/app/libs/AI2D.py for details
    def config_preprocess(self, input_image_size=None):
        # 计时器，如果debug_mode大于0则开启
        # Timer, enabled if debug_mode is greater than 0
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，可以通过设置input_image_size自行修改输入尺寸
            # Initialize ai2d preprocessing configuration, default is the size from sensor to AI,
            # can be modified by setting input_image_size
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size

            # 获取padding参数
            # Get padding parameters
            top, bottom, left, right = self.get_padding_param()

            # 填充边缘
            # Pad the edges
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [0,0,0])

            # 缩放图像，使用双线性插值
            # Resize the image using bilinear interpolation
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)

            # 构建预处理流程
            # Build the preprocessing pipeline
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                           [1,3,self.model_input_size[1],self.model_input_size[0]])

    # 自定义当前任务的后处理，results是模型输出array的列表
    # Custom post-processing for the current task, results is a list of model output arrays
    # 这里使用了aicube库的anchorbasedet_post_process接口
    # Here we use the anchorbasedet_post_process interface from the aicube library
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            # 调用aicube库进行后处理，返回检测结果
            # Call aicube library for post-processing, return detection results
            dets = aicube.anchorbasedet_post_process(
                results[0],           # 第一个输出特征图 / First output feature map
                results[1],           # 第二个输出特征图 / Second output feature map
                results[2],           # 第三个输出特征图 / Third output feature map
                self.model_input_size,# 模型输入大小 / Model input size
                self.rgb888p_size,    # 原始图像大小 / Original image size
                self.strides,         # 步长设置 / Stride settings
                len(self.labels),     # 类别数量 / Number of categories
                self.confidence_threshold, # 置信度阈值 / Confidence threshold
                self.nms_threshold,   # NMS阈值 / NMS threshold
                self.anchors,         # 锚点设置 / Anchor settings
                self.nms_option       # NMS选项 / NMS option
            )
            return dets

    # 绘制检测结果到画面上
    # Draw detection results on the screen
    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                # 清除OSD图像
                # Clear the OSD image
                pl.osd_img.clear()

                for det_box in dets:
                    # 计算显示分辨率下的坐标
                    # Calculate coordinates under display resolution
                    x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]
                    w = (x2 - x1) * self.display_size[0] // self.rgb888p_size[0]
                    h = (y2 - y1) * self.display_size[1] // self.rgb888p_size[1]
                    x1 = int(x1 * self.display_size[0] // self.rgb888p_size[0])
                    y1 = int(y1 * self.display_size[1] // self.rgb888p_size[1])
                    x2 = int(x2 * self.display_size[0] // self.rgb888p_size[0])
                    y2 = int(y2 * self.display_size[1] // self.rgb888p_size[1])

                    # 绘制矩形框和类别标签
                    # Draw rectangle box and category label
                    pl.osd_img.draw_rectangle(x1, y1, int(w), int(h), color=self.color[det_box[0]], thickness=2)
                    pl.osd_img.draw_string_advanced(
                        x1, y1-50, 32,
                        " " + self.labels[det_box[0]] + " " + str(round(det_box[1],2)),  # 类别名称和置信度 / Category name and confidence
                        color=self.color[det_box[0]]
                    )
                    # uart.send(f"${x1},{y1},{w},{h},{self.labels[det_box[0]]},{str(round(det_box[1],2))}#\n")
                    pto_data = pto.get_falldown_detect_data(x1, y1, w, h, self.labels[det_box[0]], round(det_box[1],2))
                    uart.send(pto_data)
                    print(pto_data)
            else:
                # 如果没有检测结果，清除OSD图像
                # If there are no detection results, clear the OSD image
                pl.osd_img.clear()

    # 获取padding参数
    # Get padding parameters
    def get_padding_param(self):
        # 目标宽度和高度
        # Target width and height
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]

        # 输入图像宽度和高度
        # Input image width and height
        input_width = self.rgb888p_size[0]
        input_high = self.rgb888p_size[1]

        # 计算宽高比例
        # Calculate width and height ratios
        ratio_w = dst_w / input_width
        ratio_h = dst_h / input_high

        # 选择较小的比例以保持宽高比
        # Choose the smaller ratio to maintain aspect ratio
        if ratio_w < ratio_h:
            ratio = ratio_w
        else:
            ratio = ratio_h

        # 计算新的宽高
        # Calculate new width and height
        new_w = int(ratio * input_width)
        new_h = int(ratio * input_high)

        # 计算需要填充的边距
        # Calculate margins to be padded
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2

        # 转换为整数
        # Convert to integers
        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw - 0.1))

        return top, bottom, left, right

if __name__ == "__main__":
    # 显示模式，默认"hdmi",可以选择"hdmi"和"lcd"
    # Display mode, default is "hdmi", can choose between "hdmi" and "lcd"
    display_mode="lcd"

    # 设置RGB888P图像大小
    # Set RGB888P image size
    rgb888p_size = [640,360]

    # 根据显示模式设置显示分辨率
    # Set display resolution according to display mode
    if display_mode=="hdmi":
        display_size=[1920,1080]
    else:
        display_size=[640,480]

    # 设置模型路径和其他参数
    # Set model path and other parameters
    kmodel_path = "/sdcard/kmodel/yolov5n-falldown.kmodel"  # 模型文件路径 / Model file path
    confidence_threshold = 0.3  # 置信度阈值 / Confidence threshold
    nms_threshold = 0.45  # NMS阈值 / NMS threshold
    labels = ["Fall","NoFall"]  # 模型输出类别名称 / Model output category names

    # anchor设置，用于YOLOv5检测框解码
    # Anchor settings for YOLOv5 detection box decoding
    anchors = [10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326]

    # 初始化PipeLine，用于图像处理流程
    # Initialize PipeLine for image processing workflow
    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode)
    pl.create()

    # 初始化自定义跌倒检测实例
    # Initialize custom fall detection instance
    fall_det = FallDetectionApp(
        kmodel_path,               # 模型路径 / Model path
        model_input_size=[640, 640],  # 模型输入大小 / Model input size
        labels=labels,             # 标签列表 / Label list
        anchors=anchors,           # 锚点设置 / Anchor settings
        confidence_threshold=confidence_threshold,  # 置信度阈值 / Confidence threshold
        nms_threshold=nms_threshold,  # NMS阈值 / NMS threshold
        nms_option=False,          # NMS选项 / NMS option
        strides=[8,16,32],         # 步长设置 / Stride settings
        rgb888p_size=rgb888p_size, # RGB图像大小 / RGB image size
        display_size=display_size,  # 显示大小 / Display size
        debug_mode=0               # 调试模式 / Debug mode
    )

    # 配置预处理流程
    # Configure preprocessing pipeline
    fall_det.config_preprocess()

    # 主循环，持续检测并显示结果
    # Main loop, continuously detect and display results
    while True:
        with ScopedTiming("total",0):
            # 获取当前帧数据
            # Get current frame data
            img = pl.get_frame()

            # 推理当前帧
            # Infer current frame
            res = fall_det.run(img)

            # 绘制结果到PipeLine的osd图像
            # Draw results to PipeLine's osd image
            fall_det.draw_result(pl, res)

            # 显示当前的绘制结果
            # Display current drawing results
            pl.show_image()

            # 垃圾回收，释放内存
            # Garbage collection to free memory
            gc.collect()

    # 反初始化跌倒检测实例
    # Deinitialize fall detection instance
    fall_det.deinit()

    # 销毁PipeLine实例
    # Destroy PipeLine instance
    pl.destroy()
