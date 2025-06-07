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
import aidemo
import _thread

face_det = None

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()

# 自定义人脸检测类，继承自AIBase基类
class FaceDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, anchors, confidence_threshold=0.5, nms_threshold=0.2, rgb888p_size=[224,224], display_size=[1920,1080], debug_mode=0):
        # 调用基类的构造函数 / Call parent class constructor
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)

        # 模型文件路径 / Path to the model file
        self.kmodel_path = kmodel_path

        # 模型输入分辨率 / Model input resolution
        self.model_input_size = model_input_size

        # 置信度阈值：检测结果的最小置信度要求 / Confidence threshold: minimum confidence requirement for detection results
        self.confidence_threshold = confidence_threshold

        # NMS阈值：非极大值抑制的阈值 / NMS threshold: threshold for Non-Maximum Suppression
        self.nms_threshold = nms_threshold

        # 锚点数据：用于目标检测的预定义框 / Anchor data: predefined boxes for object detection
        self.anchors = anchors

        # sensor给到AI的图像分辨率，宽度16对齐 / Image resolution from sensor to AI, width aligned to 16
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]

        # 显示分辨率，宽度16对齐 / Display resolution, width aligned to 16
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]

        # 调试模式标志 / Debug mode flag
        self.debug_mode = debug_mode

        # 实例化AI2D对象用于图像预处理 / Initialize AI2D object for image preprocessing
        self.ai2d = Ai2d(debug_mode)

        # 设置AI2D的输入输出格式 / Set AI2D input/output format
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            # 获取AI2D输入尺寸 / Get AI2D input size
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size

            # 获取padding参数 / Get padding parameters
            top, bottom, left, right = self.get_padding_param()

            # 设置padding: [上,下,左,右], 填充值[104,117,123] / Set padding: [top,bottom,left,right], padding value[104,117,123]
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [104, 117, 123])

            # 设置resize方法：双线性插值 / Set resize method: bilinear interpolation
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)

            # 构建预处理流程 / Build preprocessing pipeline
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                          [1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            # 调用aidemo库进行人脸检测后处理 / Call aidemo library for face detection post-processing
            post_ret = aidemo.face_det_post_process(self.confidence_threshold,
                                                  self.nms_threshold,
                                                  self.model_input_size[1],
                                                  self.anchors,
                                                  self.rgb888p_size,
                                                  results)
            return post_ret[0] if post_ret else post_ret

    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                # 清除上一帧的OSD绘制 / Clear previous frame's OSD drawing
                pl.osd_img.clear()

                for det in dets:
                    # 转换检测框坐标到显示分辨率 / Convert detection box coordinates to display resolution
                    x, y, w, h = map(lambda x: int(round(x, 0)), det[:4])
                    x = x * self.display_size[0] // self.rgb888p_size[0]
                    y = y * self.display_size[1] // self.rgb888p_size[1]
                    w = w * self.display_size[0] // self.rgb888p_size[0]
                    h = h * self.display_size[1] // self.rgb888p_size[1]

                    # 绘制黄色检测框 / Draw yellow detection box
                    pl.osd_img.draw_rectangle(x, y, w, h, color=(255, 255, 0, 255), thickness=2)

                    pto_data = pto.get_face_detect_data(x, y, w, h)
                    uart.send(pto_data)
                    print(pto_data)
            else:
                pl.osd_img.clear()

    def get_padding_param(self):
        # 计算模型输入和实际图像的缩放比例 / Calculate scaling ratio between model input and actual image
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]
        ratio_w = dst_w / self.rgb888p_size[0]
        ratio_h = dst_h / self.rgb888p_size[1]
        ratio = min(ratio_w, ratio_h)

        # 计算缩放后的新尺寸 / Calculate new dimensions after scaling
        new_w = int(ratio * self.rgb888p_size[0])
        new_h = int(ratio * self.rgb888p_size[1])

        # 计算padding值 / Calculate padding values
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2

        # 返回padding参数 / Return padding parameters
        return (int(round(0)),
                int(round(dh * 2 + 0.1)),
                int(round(0)),
                int(round(dw * 2 - 0.1)))

def exce_demo(pl):
    # 声明全局变量face_det / Declare global variable face_det
    global face_det

    # 获取显示相关参数 / Get display-related parameters
    display_mode = pl.display_mode      # 显示模式(如lcd) / Display mode (e.g., lcd)
    rgb888p_size = pl.rgb888p_size     # 原始图像分辨率 / Original image resolution
    display_size = pl.display_size      # 显示分辨率 / Display resolution

    # 设置人脸检测模型路径 / Set face detection model path
    kmodel_path = "/sdcard/kmodel/face_detection_320.kmodel"

    # 设置模型参数 / Set model parameters
    confidence_threshold = 0.5    # 置信度阈值 / Confidence threshold
    nms_threshold = 0.2          # 非极大值抑制阈值 / Non-maximum suppression threshold
    anchor_len = 4200            # 锚框数量 / Number of anchor boxes
    det_dim = 4                  # 检测维度(x,y,w,h) / Detection dimensions (x,y,w,h)

    # 加载锚框数据 / Load anchor box data
    anchors_path = "/sdcard/utils/prior_data_320.bin"
    anchors = np.fromfile(anchors_path, dtype=np.float)
    anchors = anchors.reshape((anchor_len, det_dim))



    try:
        # 初始化人脸检测应用实例 / Initialize face detection application instance
        face_det = FaceDetectionApp(kmodel_path,
                                  model_input_size=[320, 320],
                                  anchors=anchors,
                                  confidence_threshold=confidence_threshold,
                                  nms_threshold=nms_threshold,
                                  rgb888p_size=rgb888p_size,
                                  display_size=display_size,
                                  debug_mode=0)

        # 配置图像预处理 / Configure image preprocessing
        face_det.config_preprocess()

        # 主循环 / Main loop
        while True:
            with ScopedTiming("total",0):    # 计时器 / Timer
                img = pl.get_frame()          # 获取摄像头帧图像 / Get camera frame
                res = face_det.run(img)       # 执行人脸检测 / Run face detection
                face_det.draw_result(pl, res) # 绘制检测结果 / Draw detection results
                pl.show_image()               # 显示处理后的图像 / Display processed image
                gc.collect()                  # 垃圾回收 / Garbage collection
                time.sleep_us(10)             # 短暂延时 / Brief delay

    except Exception as e:
        print("人脸检测功能退出")           # 异常退出提示 / Exception exit prompt
    finally:
        face_det.deinit()                   # 释放资源 / Release resources

def exit_demo():
    # 程序退出时释放资源 / Release resources when program exits
    global face_det
    face_det.deinit()

if __name__ == "__main__":
    # 设置图像和显示参数 / Set image and display parameters
    rgb888p_size=[640,360]    # 原始图像分辨率 / Original image resolution
    display_size=[640,480]      # 显示分辨率 / Display resolution
    display_mode="lcd"          # 显示模式 / Display mode

    # 初始化图像处理Pipline / Initialize image processing pipeline
    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode)
    pl.create()  # 创建Pipline实例 / Create pipeline instance

    # 运行人脸检测demo / Run face detection demo
    exce_demo(pl)
