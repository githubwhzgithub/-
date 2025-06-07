# 导入必要的库 Import necessary libraries
from libs.PipeLine import PipeLine, ScopedTiming  # 导入Pipeline处理和计时器类 Import pipeline processing and timer classes
from libs.AIBase import AIBase  # 导入AI基类 Import AI base class
from libs.AI2D import Ai2d  # 导入2D图像处理类 Import 2D image processing class
from libs.Utils import *  # 导入工具函数 Import utility functions
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
import gc  # 垃圾回收模块 Garbage collection module
import sys
import aidemo

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()


# 自定义YOLOv8检测类 Custom YOLOv8 detection class
class ObjectDetectionApp(AIBase):
    def __init__(self,kmodel_path,labels,model_input_size,max_boxes_num,confidence_threshold=0.5,nms_threshold=0.2,rgb888p_size=[224,224],display_size=[1920,1080],debug_mode=0):
        """
        初始化函数 Initialization function
        kmodel_path: 模型路径 Model path
        labels: 标签列表 Label list
        model_input_size: 模型输入大小 Model input size
        max_boxes_num: 最大检测框数量 Maximum number of detection boxes
        confidence_threshold: 置信度阈值 Confidence threshold
        nms_threshold: 非极大值抑制阈值 Non-maximum suppression threshold
        rgb888p_size: RGB图像大小 RGB image size
        display_size: 显示大小 Display size
        debug_mode: 调试模式 Debug mode
        """
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path = kmodel_path
        self.labels = labels
        # 模型输入分辨率 Model input resolution
        self.model_input_size = model_input_size
        # 阈值设置 Threshold settings
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.max_boxes_num = max_boxes_num
        # sensor给到AI的图像分辨率 Image resolution from sensor to AI
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        # 显示分辨率 Display resolution
        self.display_size = [ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode = debug_mode
        # 检测框预置颜色值 Preset colors for detection boxes
        self.color_four = get_colors(len(self.labels))
        # 宽高缩放比例 Width and height scaling ratios
        self.x_factor = float(self.rgb888p_size[0])/self.model_input_size[0]
        self.y_factor = float(self.rgb888p_size[1])/self.model_input_size[1]
        # Ai2d实例，用于实现模型预处理 Ai2d instance for model preprocessing
        self.ai2d = Ai2d(debug_mode)
        # 设置Ai2d的输入输出格式和类型 Set Ai2d input/output format and type
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

    # 配置预处理操作 Configure preprocessing operations
    def config_preprocess(self,input_image_size=None):
        """
        配置图像预处理参数 Configure image preprocessing parameters
        input_image_size: 输入图像大小(可选) Input image size (optional)
        """
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            # 初始化ai2d预处理配置 Initialize ai2d preprocessing configuration
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top,bottom,left,right,self.scale = letterbox_pad_param(self.rgb888p_size,self.model_input_size)
            # 配置padding预处理 Configure padding preprocessing
            self.ai2d.pad([0,0,0,0,top,bottom,left,right], 0, [128,128,128])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    # 预处理函数 Preprocessing function
    def preprocess(self,input_np):
        with ScopedTiming("preprocess",self.debug_mode > 0):
            return [nn.from_numpy(input_np)]

    # 后处理函数 Postprocessing function
    def postprocess(self,results):
        """
        处理模型输出结果 Process model output results
        results: 模型输出 Model output
        """
        with ScopedTiming("postprocess",self.debug_mode > 0):
            new_result = results[0][0].transpose()
            det_res = aidemo.yolov8_det_postprocess(new_result.copy(),
                [self.rgb888p_size[1],self.rgb888p_size[0]],
                [self.model_input_size[1],self.model_input_size[0]],
                [self.display_size[1],self.display_size[0]],
                len(self.labels),
                self.confidence_threshold,
                self.nms_threshold,
                self.max_boxes_num)
            return det_res

    # 绘制结果函数 Draw results function
    def draw_result(self,pl,dets):
        """
        绘制检测结果 Draw detection results
        pl: PipeLine实例 PipeLine instance
        dets: 检测结果 Detection results
        """
        with ScopedTiming("display_draw",self.debug_mode >0):
            if dets:
                pl.osd_img.clear()
                for i in range(len(dets[0])):
                    x, y, w, h = map(lambda x: int(round(x, 0)), dets[0][i])
                    # 绘制矩形框和标签 Draw rectangle box and label
                    pl.osd_img.draw_rectangle(x,y, w, h, color=self.color_four[dets[1][i]],thickness=4)
                    pl.osd_img.draw_string_advanced( x , y-50,32,
                        " " + self.labels[dets[1][i]] + " " + str(round(dets[2][i],2)) ,
                        color=self.color_four[dets[1][i]])

                    pto_data = pto.get_object_detect_data(x, y, w, h, self.labels[dets[1][i]])
                    uart.send(pto_data)
                    print(pto_data)
            else:
                pl.osd_img.clear()

# 主程序入口 Main program entry
if __name__=="__main__":
    # 显示模式，默认"hdmi" Display mode, default "hdmi"
    display_mode="lcd"
    rgb888p_size=[224,224]

    # 根据显示模式设置分辨率 Set resolution based on display mode
    if display_mode=="hdmi":
        display_size=[1920,1080]
    else:
        display_size=[640,480]
    # 模型路径 Model path
    kmodel_path="/sdcard/kmodel/yolov8n_224.kmodel"
    # 标签列表 Label list
    labels = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]

    # 参数设置 Parameter settings
    confidence_threshold = 0.3  # 置信度阈值 Confidence threshold
    nms_threshold = 0.4  # NMS阈值 NMS threshold
    max_boxes_num = 30  # 最大检测框数量 Maximum number of detection boxes

    # 初始化PipeLine Initialize PipeLine
    pl = PipeLine(rgb888p_size=rgb888p_size,display_size=display_size,display_mode=display_mode)
    pl.create()

    # 初始化目标检测实例 Initialize object detection instance
    ob_det = ObjectDetectionApp(kmodel_path,labels=labels,
                              model_input_size=[224,224],
                              max_boxes_num=max_boxes_num,
                              confidence_threshold=confidence_threshold,
                              nms_threshold=nms_threshold,
                              rgb888p_size=rgb888p_size,
                              display_size=display_size,
                              debug_mode=0)
    ob_det.config_preprocess()

    # 主循环 Main loop
    while True:
        with ScopedTiming("total", 0):
            # 获取当前帧数据 Get current frame data
            img = pl.get_frame()
            # 推理当前帧 Inference current frame
            res = ob_det.run(img)
            # 绘制结果 Draw results
            ob_det.draw_result(pl,res)
            # 显示结果 Display results
            pl.show_image()
            # 垃圾回收 Garbage collection
            gc.collect()

    # 释放资源 Release resources
    ob_det.deinit()
    pl.destroy()
