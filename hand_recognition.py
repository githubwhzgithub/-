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
import image
import aicube
import random
import gc
import sys

# 自定义手掌检测任务类
# Custom hand detection task class
class HandDetApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, anchors, confidence_threshold=0.2, nms_threshold=0.5, nms_option=False, strides=[8,16,32], rgb888p_size=[1920,1080], display_size=[1920,1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        # kmodel路径
        # Path to the kmodel file
        self.kmodel_path = kmodel_path
        
        # 检测模型输入分辨率
        # Input resolution for the detection model
        self.model_input_size = model_input_size
        
        # 置信度阈值 - 仅保留高于此阈值的检测结果
        # Confidence threshold - only keep detection results above this threshold
        self.confidence_threshold = confidence_threshold
        
        # nms阈值 - 用于非极大值抑制的IOU阈值
        # NMS threshold - IOU threshold used for non-maximum suppression
        self.nms_threshold = nms_threshold
        
        # 锚框,目标检测任务使用
        # Anchor boxes used for object detection tasks
        self.anchors = anchors
        
        # 特征下采样倍数 - 对应于不同特征图层的步长
        # Feature downsampling factors - corresponding to strides for different feature map layers
        self.strides = strides
        
        # NMS选项，如果为True做类间NMS,如果为False做类内NMS
        # NMS option, if True performs inter-class NMS, if False performs intra-class NMS
        self.nms_option = nms_option
        
        # sensor给到AI的图像分辨率，宽16字节对齐
        # Image resolution from sensor to AI, width aligned to 16 bytes
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        
        # 视频输出VO分辨率，宽16字节对齐
        # Video output resolution, width aligned to 16 bytes
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        
        # debug模式 - 控制是否打印调试信息
        # Debug mode - controls whether to print debug information
        self.debug_mode = debug_mode
        
        # Ai2d实例用于实现预处理
        # Ai2d instance for implementing preprocessing
        self.ai2d = Ai2d(debug_mode)
        
        # 设置ai2d的输入输出的格式和数据类型
        # Set input/output format and data type for ai2d
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    # 配置预处理操作，这里使用了pad和resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/app/libs/AI2D.py查看
    # Configure preprocessing operations, using pad and resize here. Ai2d supports crop/shift/pad/resize/affine, see /sdcard/app/libs/AI2D.py for details
    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，可以通过设置input_image_size自行修改输入尺寸
            # Initialize ai2d preprocessing configuration, default is the size from sensor to AI, can be modified by setting input_image_size
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            
            # 计算padding参数并应用pad操作，以确保输入图像尺寸与模型输入尺寸匹配
            # Calculate padding parameters and apply pad operation to ensure input image size matches model input size
            top, bottom, left, right = self.get_padding_param()
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
            
            # 使用双线性插值进行resize操作，调整图像尺寸以符合模型输入要求
            # Use bilinear interpolation for resize operation to adjust image size to meet model input requirements
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            
            # 构建预处理流程,参数为预处理输入tensor的shape和预处理输出的tensor的shape
            # Build preprocessing pipeline, parameters are the shape of preprocessing input tensor and output tensor
            self.ai2d.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]], [1, 3, self.model_input_size[1], self.model_input_size[0]])

    # 自定义当前任务的后处理，用于处理模型输出结果，这里使用了aicube库的anchorbasedet_post_process接口
    # Custom post-processing for the current task, used to process model output results, using anchorbasedet_post_process interface from aicube library
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            dets = aicube.anchorbasedet_post_process(results[0], results[1], results[2], self.model_input_size, self.rgb888p_size, self.strides, 1, self.confidence_threshold, self.nms_threshold, self.anchors, self.nms_option)
            # 返回手掌检测结果
            # Return hand detection results
            return dets

    # 计算padding参数，确保输入图像尺寸与模型输入尺寸匹配
    # Calculate padding parameters to ensure input image size matches model input size
    def get_padding_param(self):
        # 根据目标宽度和高度计算比例因子
        # Calculate scaling factors based on target width and height
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]
        input_width = self.rgb888p_size[0]
        input_high = self.rgb888p_size[1]
        ratio_w = dst_w / input_width
        ratio_h = dst_h / input_high
        
        # 选择较小的比例因子，以确保图像内容完整
        # Choose the smaller scaling factor to ensure image content is complete
        if ratio_w < ratio_h:
            ratio = ratio_w
        else:
            ratio = ratio_h
            
        # 计算新的宽度和高度
        # Calculate new width and height
        new_w = int(ratio * input_width)
        new_h = int(ratio * input_high)
        
        # 计算宽度和高度的差值，并确定padding的位置
        # Calculate width and height differences and determine padding positions
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw + 0.1))
        return top, bottom, left, right

# 自定义手势识别任务类
# Custom hand gesture recognition task class
class HandRecognitionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, labels, rgb888p_size=[1920,1080], display_size=[1920,1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        # kmodel路径
        # Path to the kmodel file
        self.kmodel_path = kmodel_path
        
        # 检测模型输入分辨率
        # Input resolution for the detection model
        self.model_input_size = model_input_size
        
        # 标签列表，用于识别不同的手势
        # Label list for recognizing different gestures
        self.labels = labels
        
        # sensor给到AI的图像分辨率，宽16字节对齐
        # Image resolution from sensor to AI, width aligned to 16 bytes
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        
        # 视频输出VO分辨率，宽16字节对齐
        # Video output resolution, width aligned to 16 bytes
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        
        # 裁剪参数，用于从原始图像中裁剪出手部区域
        # Crop parameters for cropping hand regions from the original image
        self.crop_params = []
        
        # debug模式
        # Debug mode
        self.debug_mode = debug_mode
        
        # Ai2d实例用于实现预处理
        # Ai2d instance for implementing preprocessing
        self.ai2d = Ai2d(debug_mode)
        
        # 设置ai2d的输入输出的格式和数据类型
        # Set input/output format and data type for ai2d
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    # 配置预处理操作，这里使用了crop和resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/app/libs/AI2D.py查看
    # Configure preprocessing operations, using crop and resize here. Ai2d supports crop/shift/pad/resize/affine, see /sdcard/app/libs/AI2D.py for details
    def config_preprocess(self, det, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            
            # 根据检测框计算裁剪参数
            # Calculate crop parameters based on detection box
            self.crop_params = self.get_crop_param(det)
            
            # 应用裁剪操作
            # Apply crop operation
            self.ai2d.crop(self.crop_params[0], self.crop_params[1], self.crop_params[2], self.crop_params[3])
            
            # 使用双线性插值进行resize操作
            # Use bilinear interpolation for resize operation
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            
            # 构建预处理流程
            # Build preprocessing pipeline
            self.ai2d.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]], [1, 3, self.model_input_size[1], self.model_input_size[0]])

    # 自定义后处理，results是模型输出的array列表
    # Custom post-processing, results is a list of arrays output by the model
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            # 重塑模型输出结果
            # Reshape model output results
            result = results[0].reshape(results[0].shape[0] * results[0].shape[1])
            
            # 应用softmax函数获取概率分布
            # Apply softmax function to get probability distribution
            x_softmax = self.softmax(result)
            
            # 获取最大概率的索引（即预测的手势类别）
            # Get the index with maximum probability (i.e., predicted gesture class)
            idx = np.argmax(x_softmax)
            
            # 构建结果文本，包含预测的手势标签和概率
            # Build result text, including predicted gesture label and probability
            text = " " + self.labels[idx] + ": " + str(round(x_softmax[idx], 2))
            return text

    # 计算crop参数
    # Calculate crop parameters
    def get_crop_param(self, det_box):
        # 获取检测框的坐标
        # Get coordinates of detection box
        x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]
        w, h = int(x2 - x1), int(y2 - y1)
        
        # 计算检测框在显示尺寸上的宽高和坐标
        # Calculate width, height and coordinates of detection box in display size
        w_det = int(float(x2 - x1) * self.display_size[0] // self.rgb888p_size[0])
        h_det = int(float(y2 - y1) * self.display_size[1] // self.rgb888p_size[1])
        x_det = int(x1 * self.display_size[0] // self.rgb888p_size[0])
        y_det = int(y1 * self.display_size[1] // self.rgb888p_size[1])
        
        # 扩大裁剪区域以包含整个手部
        # Expand cropping area to include the entire hand
        length = max(w, h) / 2
        cx = (x1 + x2) / 2   # 中心点x坐标 / Center point x-coordinate
        cy = (y1 + y2) / 2   # 中心点y坐标 / Center point y-coordinate
        ratio_num = 1.26 * length   # 扩展系数 / Expansion coefficient
        
        # 计算扩展后的裁剪区域，并确保在图像范围内
        # Calculate expanded cropping area and ensure it's within image bounds
        x1_kp = int(max(0, cx - ratio_num))
        y1_kp = int(max(0, cy - ratio_num))
        x2_kp = int(min(self.rgb888p_size[0] - 1, cx + ratio_num))
        y2_kp = int(min(self.rgb888p_size[1] - 1, cy + ratio_num))
        w_kp = int(x2_kp - x1_kp + 1)
        h_kp = int(y2_kp - y1_kp + 1)
        
        return [x1_kp, y1_kp, w_kp, h_kp]

    # softmax实现 - 将模型输出转换为概率分布
    # Softmax implementation - convert model output to probability distribution
    def softmax(self, x):
        # 为了数值稳定性，减去最大值
        # For numerical stability, subtract maximum value
        x -= np.max(x)
        
        # 计算指数和归一化
        # Calculate exponential and normalize
        x = np.exp(x) / np.sum(np.exp(x))
        return x

# 手势识别主类，整合了手掌检测和手势识别
# Hand gesture recognition main class, integrating hand detection and gesture recognition
class HandRecognition:
    def __init__(self, hand_det_kmodel, hand_kp_kmodel, det_input_size, kp_input_size, labels, anchors, confidence_threshold=0.25, nms_threshold=0.3, nms_option=False, strides=[8,16,32], rgb888p_size=[1280,720], display_size=[1920,1080], debug_mode=0):
        # 手掌检测模型路径
        # Path to hand detection model
        self.hand_det_kmodel = hand_det_kmodel
        
        # 手掌关键点模型路径
        # Path to hand keypoint model
        self.hand_kp_kmodel = hand_kp_kmodel
        
        # 手掌检测模型输入分辨率
        # Hand detection model input resolution
        self.det_input_size = det_input_size
        
        # 手掌关键点模型输入分辨率
        # Hand keypoint model input resolution
        self.kp_input_size = kp_input_size
        
        # 手势标签列表
        # Gesture label list
        self.labels = labels
        
        # anchors参数，用于检测模型
        # Anchors parameters for detection model
        self.anchors = anchors
        
        # 置信度阈值
        # Confidence threshold
        self.confidence_threshold = confidence_threshold
        
        # nms阈值
        # NMS threshold
        self.nms_threshold = nms_threshold
        
        # nms选项
        # NMS option
        self.nms_option = nms_option
        
        # 特征图针对输出的下采样倍数
        # Downsampling factors for feature maps relative to output
        self.strides = strides
        
        # sensor给到AI的图像分辨率，宽16字节对齐
        # Image resolution from sensor to AI, width aligned to 16 bytes
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        
        # 视频输出VO分辨率，宽16字节对齐
        # Video output resolution, width aligned to 16 bytes
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        
        # debug_mode模式
        # Debug mode
        self.debug_mode = debug_mode
        
        # 创建手掌检测和手势识别实例
        # Create instances for hand detection and gesture recognition
        self.hand_det = HandDetApp(self.hand_det_kmodel, model_input_size=self.det_input_size, anchors=self.anchors, 
                                   confidence_threshold=self.confidence_threshold, nms_threshold=self.nms_threshold, 
                                   nms_option=self.nms_option, strides=self.strides, rgb888p_size=self.rgb888p_size, 
                                   display_size=self.display_size, debug_mode=0)
        
        self.hand_rec = HandRecognitionApp(self.hand_kp_kmodel, model_input_size=self.kp_input_size, labels=self.labels, 
                                          rgb888p_size=self.rgb888p_size, display_size=self.display_size)
        
        # 配置手掌检测的预处理
        # Configure preprocessing for hand detection
        self.hand_det.config_preprocess()

    # run函数 - 执行手掌检测和手势识别的主流程
    # Run function - main process for hand detection and gesture recognition
    def run(self, input_np):
        # 执行手掌检测
        # Perform hand detection
        det_boxes = self.hand_det.run(input_np)
        
        hand_rec_res = []  # 存储手势识别结果 / Store gesture recognition results
        hand_det_res = []  # 存储有效的手掌检测结果 / Store valid hand detection results
        
        for det_box in det_boxes:
            # 对检测到的每一个手掌执行手势识别
            # Perform gesture recognition for each detected hand
            x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]
            w, h = int(x2 - x1), int(y2 - y1)
            
            # 过滤掉一些不合理的检测框
            # Filter out unreasonable detection boxes
            
            # 过滤高度太小的检测框 / Filter detection boxes with too small height
            if (h < (0.1 * self.rgb888p_size[1])):
                continue
                
            # 过滤在边缘且宽度较小的检测框 / Filter detection boxes at edges with small width
            if (w < (0.25 * self.rgb888p_size[0]) and ((x1 < (0.03 * self.rgb888p_size[0])) or (x2 > (0.97 * self.rgb888p_size[0])))):
                continue
                
            # 过滤在极端边缘且宽度很小的检测框 / Filter detection boxes at extreme edges with very small width
            if (w < (0.15 * self.rgb888p_size[0]) and ((x1 < (0.01 * self.rgb888p_size[0])) or (x2 > (0.99 * self.rgb888p_size[0])))):
                continue
                
            # 为当前检测框配置预处理并执行手势识别
            # Configure preprocessing for current detection box and perform gesture recognition
            self.hand_rec.config_preprocess(det_box)
            text = self.hand_rec.run(input_np)
            
            # 保存有效的检测结果和识别结果
            # Save valid detection and recognition results
            hand_det_res.append(det_box)
            hand_rec_res.append(text)
            
        return hand_det_res, hand_rec_res

    # 绘制效果，绘制识别结果和检测框
    # Draw results, including recognition results and detection boxes
    def draw_result(self, pl, hand_det_res, hand_rec_res):
        # 清除OSD图像上的内容
        # Clear content on OSD image
        pl.osd_img.clear()
        
        if hand_det_res:
            for k in range(len(hand_det_res)):
                det_box = hand_det_res[k]
                x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]
                w, h = int(x2 - x1), int(y2 - y1)
                
                # 计算检测框在显示尺寸上的宽高和坐标
                # Calculate width, height and coordinates of detection box in display size
                w_det = int(float(x2 - x1) * self.display_size[0] // self.rgb888p_size[0])
                h_det = int(float(y2 - y1) * self.display_size[1] // self.rgb888p_size[1])
                x_det = int(x1 * self.display_size[0] // self.rgb888p_size[0])
                y_det = int(y1 * self.display_size[1] // self.rgb888p_size[1])
                
                # 绘制矩形框和识别结果文本
                # Draw rectangle box and recognition result text
                pl.osd_img.draw_rectangle(x_det, y_det, w_det, h_det, color=(255, 0, 255, 0), thickness=2)
                pl.osd_img.draw_string_advanced(x_det, y_det-50, 32, hand_rec_res[k], color=(255, 0, 255, 0))


if __name__=="__main__":
    # 显示模式，默认"hdmi",可以选择"hdmi"和"lcd"
    # Display mode, default is "hdmi", can choose between "hdmi" and "lcd"
    display_mode = "lcd"
    rgb888p_size = [640, 360]

    if display_mode == "hdmi":
        display_size = [1920, 1080]
    else:
        display_size = [640, 480]
        
    # 手掌检测模型路径
    # Path to hand detection model
    hand_det_kmodel_path = "/sdcard/kmodel/hand_det.kmodel"
    
    # 手势识别模型路径
    # Path to hand gesture recognition model
    hand_rec_kmodel_path = "/sdcard/kmodel/hand_reco.kmodel"
    
    # 其它参数
    # Other parameters
    anchors_path = "/sdcard/utils/prior_data_320.bin"
    hand_det_input_size = [512, 512]
    hand_rec_input_size = [224, 224]
    confidence_threshold = 0.2
    nms_threshold = 0.5
    labels = ["gun", "other", "yeah", "five"]
    anchors = [26, 27, 53, 52, 75, 71, 80, 99, 106, 82, 99, 134, 140, 113, 161, 172, 245, 276]

    # 初始化PipeLine，只关注传给AI的图像分辨率，显示的分辨率
    # Initialize PipeLine, only focus on image resolution passed to AI and display resolution
    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode)
    pl.create()
    
    # 创建手势识别实例
    # Create hand gesture recognition instance
    hr = HandRecognition(hand_det_kmodel_path, hand_rec_kmodel_path, det_input_size=hand_det_input_size, 
                        kp_input_size=hand_rec_input_size, labels=labels, anchors=anchors, 
                        confidence_threshold=confidence_threshold, nms_threshold=nms_threshold, 
                        nms_option=False, strides=[8, 16, 32], rgb888p_size=rgb888p_size, display_size=display_size)
    
    # 主循环
    # Main loop
    while True:
        with ScopedTiming("total", 1):
            img = pl.get_frame()                              # 获取当前帧 / Get current frame
            hand_det_res, hand_rec_res = hr.run(img)         # 推理当前帧 / Inference on current frame
            hr.draw_result(pl, hand_det_res, hand_rec_res)    # 绘制推理结果 / Draw inference results
            pl.show_image()                                 # 展示推理结果 / Display inference results
            gc.collect()                                    # 进行垃圾回收 / Perform garbage collection
            
    # 资源释放
    # Resource release
    hr.hand_det.deinit()
    hr.hand_rec.deinit()
    pl.destroy()