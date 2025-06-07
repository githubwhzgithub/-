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

# 自定义手掌检测类，继承自AIBase基类
# Custom hand detection class that inherits from AIBase base class
class HandDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, labels, anchors, confidence_threshold=0.2, nms_threshold=0.5, nms_option=False, strides=[8,16,32], rgb888p_size=[224,224], display_size=[1920,1080], debug_mode=0):
        # 调用基类的构造函数，初始化模型文件路径、模型输入分辨率、RGB图像分辨率和调试模式
        # Call the constructor of the parent class to initialize the model file path, model input resolution, RGB image resolution, and debug mode
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)

        # 模型文件路径
        # Model file path
        self.kmodel_path = kmodel_path

        # 模型输入分辨率
        # Model input resolution
        self.model_input_size = model_input_size

        # 模型输出的类别标签列表
        # List of class labels for model output
        self.labels = labels

        # 用于目标检测的锚点尺寸列表
        # List of anchor sizes for object detection
        self.anchors = anchors

        # 特征下采样倍数
        # Feature downsampling ratio
        self.strides = strides

        # 置信度阈值，用于过滤低置信度的检测结果
        # Confidence threshold for filtering low-confidence detection results
        self.confidence_threshold = confidence_threshold

        # NMS（非极大值抑制）阈值，用于去除重叠的检测框
        # NMS (Non-Maximum Suppression) threshold for removing overlapping detection boxes
        self.nms_threshold = nms_threshold

        # NMS选项，可能影响NMS的具体实现
        # NMS options that may affect the specific implementation of NMS
        self.nms_option = nms_option

        # sensor给到AI的图像分辨率，对齐到最近的16的倍数
        # Image resolution from sensor to AI, aligned to the nearest multiple of 16
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]

        # 显示分辨率，对齐到最近的16的倍数
        # Display resolution, aligned to the nearest multiple of 16
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]

        # 调试模式，用于输出调试信息
        # Debug mode, used for outputting debug information
        self.debug_mode = debug_mode

        # 实例化Ai2d类，用于实现模型预处理
        # Instantiate the Ai2d class for model preprocessing
        self.ai2d = Ai2d(debug_mode)

        # 设置Ai2d的输入输出格式和类型，这里使用NCHW格式，数据类型为uint8
        # Set the input and output format and type for Ai2d, using NCHW format and uint8 data type
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    # 配置预处理操作，这里使用了pad和resize
    # Configure preprocessing operations, using pad and resize here
    def config_preprocess(self, input_image_size=None):
        # 使用ScopedTiming装饰器来测量预处理配置的时间
        # Use the ScopedTiming decorator to measure the time of preprocessing configuration
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，可以通过设置input_image_size自行修改输入尺寸
            # Initialize the ai2d preprocessing configuration, default is the size from sensor to AI,
            # can be modified by setting input_image_size
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size

            # 计算padding参数并应用pad操作，以确保输入图像尺寸与模型输入尺寸匹配
            # Calculate the padding parameters and apply the pad operation to ensure that the input image size matches the model input size
            top, bottom, left, right = self.get_padding_param()
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [0, 0, 0])

            # 使用双线性插值进行resize操作，调整图像尺寸以符合模型输入要求
            # Use bilinear interpolation for the resize operation to adjust the image size to meet the model input requirement
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)

            # 构建预处理流程
            # Build the preprocessing pipeline
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    # 自定义当前任务的后处理，用于处理模型输出结果
    # Custom post-processing for the current task, used to process the model output results
    def postprocess(self, results):
        # 使用ScopedTiming装饰器来测量后处理的时间
        # Use the ScopedTiming decorator to measure the time of post-processing
        with ScopedTiming("postprocess", self.debug_mode > 0):
            # 使用aicube库的函数进行后处理，得到最终的检测结果
            # Use the function in the aicube library for post-processing to get the final detection results
            dets = aicube.anchorbasedet_post_process(results[0], results[1], results[2], self.model_input_size, self.rgb888p_size, self.strides, len(self.labels), self.confidence_threshold, self.nms_threshold, self.anchors, self.nms_option)
            return dets

    # 绘制检测结果到屏幕上
    # Draw the detection results on the screen
    def draw_result(self, pl, dets):
        # 使用ScopedTiming装饰器来测量绘制结果的时间
        # Use the ScopedTiming decorator to measure the time of drawing results
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:  # 如果存在检测结果
                pl.osd_img.clear()  # 清除屏幕上的旧内容
                for det_box in dets:  # 遍历每个检测框
                    # 根据模型输出计算检测框的像素坐标，并调整大小以适应显示分辨率
                    # Calculate the pixel coordinates of the detection box based on the model output,
                    # and adjust the size to fit the display resolution
                    x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]
                    w = float(x2 - x1) * self.display_size[0] // self.rgb888p_size[0]
                    h = float(y2 - y1) * self.display_size[1] // self.rgb888p_size[1]
                    x1 = int(x1 * self.display_size[0] // self.rgb888p_size[0])
                    y1 = int(y1 * self.display_size[1] // self.rgb888p_size[1])
                    x2 = int(x2 * self.display_size[0] // self.rgb888p_size[0])
                    y2 = int(y2 * self.display_size[1] // self.rgb888p_size[1])

                    # 过滤掉太小或者位置不合理的检测框
                    # Filter out detection boxes that are too small or in unreasonable positions
                    if (h < (0.1 * self.display_size[0])):
                        continue
                    if (w < (0.25 * self.display_size[0]) and ((x1 < (0.03 * self.display_size[0])) or (x2 > (0.97 * self.display_size[0])))):
                        continue
                    if (w < (0.15 * self.display_size[0]) and ((x1 < (0.01 * self.display_size[0])) or (x2 > (0.99 * self.display_size[0])))):
                        continue

                    # 绘制矩形框和类别标签
                    # Draw the rectangle box and class label
                    pl.osd_img.draw_rectangle(x1, y1, int(w), int(h), color=(255, 0, 255, 0), thickness=2)
                    pl.osd_img.draw_string_advanced(x1, y1-50,32, " " + self.labels[det_box[0]] + " " + str(round(det_box[1], 2)), color=(255, 0, 255, 0))
                    pto_data = pto.get_hand_detect_data(x1, y1, w, h)
                    uart.send(pto_data)
                    print(pto_data)
            else:
                pl.osd_img.clear()  # 如果没有检测结果，清空屏幕

    # 计算padding参数，确保输入图像尺寸与模型输入尺寸匹配
    # Calculate the padding parameters to ensure that the input image size matches the model input size
    def get_padding_param(self):
        # 根据目标宽度和高度计算比例因子
        # Calculate the scaling factor based on the target width and height
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]
        input_width = self.rgb888p_size[0]
        input_high = self.rgb888p_size[1]
        ratio_w = dst_w / input_width
        ratio_h = dst_h / input_high

        # 选择较小的比例因子，以确保图像内容完整
        # Choose the smaller scaling factor to ensure the integrity of the image content
        if ratio_w < ratio_h:
            ratio = ratio_w
        else:
            ratio = ratio_h

        # 计算新的宽度和高度
        # Calculate the new width and height
        new_w = int(ratio * input_width)
        new_h = int(ratio * input_high)

        # 计算宽度和高度的差值，并确定padding的位置
        # Calculate the difference in width and height, and determine the position of the padding
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw + 0.1))
        return top, bottom, left, right

if __name__=="__main__":
    # 显示模式，默认"hdmi",可以选择"hdmi"和"lcd"
    # Display mode, default is "hdmi", can choose between "hdmi" and "lcd"
    display_mode="lcd"
    # k230保持不变，k230d可调整为[640,360]
    # Keep k230 unchanged, k230d can be adjusted to [640,360]
    rgb888p_size = [640,360]

    # 根据显示模式设置显示分辨率
    # Set the display resolution according to the display mode
    if display_mode=="hdmi":
        display_size=[1920,1080]
    else:
        display_size=[640,480]

    # 模型路径
    # Model path
    kmodel_path="/sdcard/kmodel/hand_det.kmodel"

    # 其它参数设置
    # Other parameter settings
    confidence_threshold = 0.2
    nms_threshold = 0.5
    labels = ["hand"]
    anchors = [26,27, 53,52, 75,71, 80,99, 106,82, 99,134, 140,113, 161,172, 245,276]   #anchor设置

    # 初始化PipeLine
    # Initialize PipeLine
    pl=PipeLine(rgb888p_size=rgb888p_size,display_size=display_size,display_mode=display_mode)
    pl.create()

    # 初始化自定义手掌检测实例
    # Initialize custom hand detection instance
    hand_det=HandDetectionApp(kmodel_path,model_input_size=[512,512],labels=labels,anchors=anchors,confidence_threshold=confidence_threshold,nms_threshold=nms_threshold,nms_option=False,strides=[8,16,32],rgb888p_size=rgb888p_size,display_size=display_size,debug_mode=0)
    hand_det.config_preprocess()

    # 主循环，持续检测并显示结果
    # Main loop, continuously detect and display results
    while True:
        with ScopedTiming("total",0):
            # 获取当前帧数据
            # Get current frame data
            img=pl.get_frame()

            # 推理当前帧
            # Infer current frame
            res=hand_det.run(img)

            # 绘制结果到PipeLine的osd图像
            # Draw results to PipeLine's osd image
            hand_det.draw_result(pl,res)

            # 显示当前的绘制结果
            # Display current drawing results
            pl.show_image()

            # 垃圾回收
            # Garbage collection
            gc.collect()

    # 反初始化
    # Deinitialize
    hand_det.deinit()

    # 销毁PipeLine实例
    # Destroy PipeLine instance
    pl.destroy()
