# 导入所需库 / Import required libraries
from libs.PipeLine import PipeLine, ScopedTiming  # 导入Pipeline和计时工具 / Import pipeline and timing tools
from libs.AIBase import AIBase  # 导入AI基类 / Import AI base class
from libs.AI2D import Ai2d  # 导入AI 2D处理工具 / Import AI 2D processing tool
from random import randint  # 导入随机数生成器 / Import random number generator
import os
import ujson
from media.media import *
from time import *
import nncase_runtime as nn  # 导入神经网络运行时 / Import neural network runtime
import ulab.numpy as np  # 导入numpy库 / Import numpy library
import time
import image
import aicube  # 导入AI立方体处理库 / Import AI cube processing library
import random
import gc  # 导入垃圾回收器 / Import garbage collector
import sys
import machine

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()

dg = None

class HandDetApp(AIBase):

    """
    手掌检测应用类 / Hand Detection Application Class
    继承自AIBase基类 / Inherits from AIBase class
    """
    def __init__(self, kmodel_path, labels, model_input_size, anchors,
                 confidence_threshold=0.2, nms_threshold=0.5, nms_option=False,
                 strides=[8,16,32], rgb888p_size=[1920,1080],
                 display_size=[1920,1080], debug_mode=0):
        """
        初始化函数 / Initialization function

        参数 / Parameters:
        kmodel_path: 模型文件路径 / Path to the model file
        labels: 检测标签列表 / List of detection labels
        model_input_size: 模型输入尺寸 / Model input size
        anchors: 锚框设置 / Anchor box settings
        confidence_threshold: 置信度阈值 / Confidence threshold
        nms_threshold: NMS阈值 / NMS threshold
        nms_option: NMS选项(类间/类内) / NMS option (inter-class/intra-class)
        strides: 特征图下采样倍数 / Feature map downsampling factors
        rgb888p_size: 输入图像尺寸 / Input image size
        display_size: 显示尺寸 / Display size
        debug_mode: 调试模式 / Debug mode
        """
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)

        self.kmodel_path = kmodel_path
        self.labels = labels
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.anchors = anchors
        self.strides = strides
        self.nms_option = nms_option
        # 确保宽度16字节对齐 / Ensure width is aligned to 16 bytes
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.debug_mode = debug_mode

        # 初始化AI2D处理器 / Initialize AI2D processor
        self.ai2d = Ai2d(debug_mode)
        # 设置AI2D数据格式和类型 / Set AI2D data format and type
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,
                                nn.ai2d_format.NCHW_FMT,
                                np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        """
        配置图像预处理流程 / Configure image preprocessing pipeline

        参数 / Parameter:
        input_image_size: 输入图像尺寸，若为None则使用默认尺寸 /
                         Input image size, use default size if None
        """
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size

            # 计算padding参数 / Calculate padding parameters
            top, bottom, left, right = self.get_padding_param()
            # 设置padding / Set padding
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
            # 设置resize方法 / Set resize method
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            # 构建处理流程 / Build processing pipeline
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])
    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            dets = aicube.anchorbasedet_post_process(results[0], results[1], results[2], self.model_input_size, self.rgb888p_size, self.strides, len(self.labels), self.confidence_threshold, self.nms_threshold, self.anchors, self.nms_option)
            return dets

    def get_padding_param(self):
        """
        计算padding参数以保持图像比例 / Calculate padding parameters to maintain image ratio

        返回值 / Returns:
        top, bottom, left, right: padding的像素数 / Number of pixels for padding
        """
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]
        input_width = self.rgb888p_size[0]
        input_high = self.rgb888p_size[1]

        # 计算宽高比例 / Calculate width and height ratio
        ratio_w = dst_w / input_width
        ratio_h = dst_h / input_high
        ratio = min(ratio_w, ratio_h)

        # 计算新的尺寸 / Calculate new dimensions
        new_w = int(ratio * input_width)
        new_h = int(ratio * input_high)

        # 计算padding值 / Calculate padding values
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw + 0.1))

        return top, bottom, left, right

class HandKPClassApp(AIBase):
    """
    手掌关键点分类应用类 / Hand Keypoint Classification Application Class
    用于检测手掌关键点并识别手势 / Used for detecting hand keypoints and recognizing gestures
    """
    def __init__(self, kmodel_path, model_input_size, rgb888p_size=[1920,1080],
                 display_size=[1920,1080], debug_mode=0):
        """
        初始化函数 / Initialization function

        参数 / Parameters:
        kmodel_path: 模型文件路径 / Path to the model file
        model_input_size: 模型输入尺寸 / Model input size
        rgb888p_size: 原始图像尺寸 / Original image size
        display_size: 显示尺寸 / Display size
        debug_mode: 调试模式 / Debug mode
        """
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)

        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        # 确保图像宽度16字节对齐 / Ensure image width is aligned to 16 bytes
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        # 存储裁剪参数 / Store cropping parameters
        self.crop_params = []
        self.debug_mode = debug_mode

        # 初始化AI2D处理器 / Initialize AI2D processor
        self.ai2d = Ai2d(debug_mode)
        # 设置AI2D数据格式 / Set AI2D data format
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,
                                nn.ai2d_format.NCHW_FMT,
                                np.uint8, np.uint8)

    def config_preprocess(self, det, input_image_size=None):
        """
        配置预处理参数 / Configure preprocessing parameters

        参数 / Parameters:
        det: 检测框参数 / Detection box parameters
        input_image_size: 输入图像尺寸 / Input image size
        """
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            # 设置输入尺寸 / Set input size
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size

            # 获取并设置裁剪参数 / Get and set cropping parameters
            self.crop_params = self.get_crop_param(det)
            self.ai2d.crop(self.crop_params[0], self.crop_params[1],
                          self.crop_params[2], self.crop_params[3])

            # 设置缩放方法 / Set resize method
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)

            # 构建预处理Pipeline / Build preprocessing pipeline
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                          [1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self, results):
        """
        后处理函数 / Post-processing function

        参数 / Parameters:
        results: 模型输出结果 / Model output results

        返回 / Returns:
        results_show: 处理后的关键点坐标 / Processed keypoint coordinates
        gesture: 识别出的手势 / Recognized gesture
        """
        with ScopedTiming("postprocess", self.debug_mode > 0):
            # 重塑输出数组 / Reshape output array
            results = results[0].reshape(results[0].shape[0]*results[0].shape[1])
            results_show = np.zeros(results.shape, dtype=np.int16)

            # 转换坐标到原始图像空间 / Convert coordinates to original image space
            results_show[0::2] = results[0::2] * self.crop_params[3] + self.crop_params[0]
            results_show[1::2] = results[1::2] * self.crop_params[2] + self.crop_params[1]
            # 计算手势类型 / Calculate gesture type
            gesture = self.hk_gesture(results_show)
            return results_show, gesture
    def get_crop_param(self,det_box):
        x1, y1, x2, y2 = det_box[2],det_box[3],det_box[4],det_box[5]
        w,h= int(x2 - x1),int(y2 - y1)
        w_det = int(float(x2 - x1) * self.display_size[0] // self.rgb888p_size[0])
        h_det = int(float(y2 - y1) * self.display_size[1] // self.rgb888p_size[1])
        x_det = int(x1*self.display_size[0] // self.rgb888p_size[0])
        y_det = int(y1*self.display_size[1] // self.rgb888p_size[1])
        length = max(w, h)/2
        cx = (x1+x2)/2
        cy = (y1+y2)/2
        ratio_num = 1.26*length
        x1_kp = int(max(0,cx-ratio_num))
        y1_kp = int(max(0,cy-ratio_num))
        x2_kp = int(min(self.rgb888p_size[0]-1, cx+ratio_num))
        y2_kp = int(min(self.rgb888p_size[1]-1, cy+ratio_num))
        w_kp = int(x2_kp - x1_kp + 1)
        h_kp = int(y2_kp - y1_kp + 1)
        return [x1_kp, y1_kp, w_kp, h_kp]

    # 求两个vector之间的夹角
    def hk_vector_2d_angle(self,v1,v2):
        with ScopedTiming("hk_vector_2d_angle",self.debug_mode > 0):
            v1_x,v1_y,v2_x,v2_y = v1[0],v1[1],v2[0],v2[1]
            v1_norm = np.sqrt(v1_x * v1_x+ v1_y * v1_y)
            v2_norm = np.sqrt(v2_x * v2_x + v2_y * v2_y)
            dot_product = v1_x * v2_x + v1_y * v2_y
            cos_angle = dot_product/(v1_norm*v2_norm)
            angle = np.acos(cos_angle)*180/np.pi
            return angle

    # 根据手掌关键点检测结果判断手势类别
    def hk_gesture(self,results):
        with ScopedTiming("hk_gesture",self.debug_mode > 0):
            angle_list = []
            for i in range(5):
                angle = self.hk_vector_2d_angle([(results[0]-results[i*8+4]), (results[1]-results[i*8+5])],[(results[i*8+6]-results[i*8+8]),(results[i*8+7]-results[i*8+9])])
                angle_list.append(angle)
            thr_angle,thr_angle_thumb,thr_angle_s,gesture_str = 65.,53.,49.,None
            if 65535. not in angle_list:
                if (angle_list[0]>thr_angle_thumb)  and (angle_list[1]>thr_angle) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
                    gesture_str = "fist"
                elif (angle_list[0]<thr_angle_s)  and (angle_list[1]<thr_angle_s) and (angle_list[2]<thr_angle_s) and (angle_list[3]<thr_angle_s) and (angle_list[4]<thr_angle_s):
                    gesture_str = "five"
                elif (angle_list[0]<thr_angle_s)  and (angle_list[1]<thr_angle_s) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
                    gesture_str = "gun"
                elif (angle_list[0]<thr_angle_s)  and (angle_list[1]<thr_angle_s) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]<thr_angle_s):
                    gesture_str = "love"
                elif (angle_list[0]>5)  and (angle_list[1]<thr_angle_s) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
                    gesture_str = "one"
                elif (angle_list[0]<thr_angle_s)  and (angle_list[1]>thr_angle) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]<thr_angle_s):
                    gesture_str = "six"
                elif (angle_list[0]>thr_angle_thumb)  and (angle_list[1]<thr_angle_s) and (angle_list[2]<thr_angle_s) and (angle_list[3]<thr_angle_s) and (angle_list[4]>thr_angle):
                    gesture_str = "three"
                elif (angle_list[0]<thr_angle_s)  and (angle_list[1]>thr_angle) and (angle_list[2]>thr_angle) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
                    gesture_str = "thumbUp"
                elif (angle_list[0]>thr_angle_thumb)  and (angle_list[1]<thr_angle_s) and (angle_list[2]<thr_angle_s) and (angle_list[3]>thr_angle) and (angle_list[4]>thr_angle):
                    gesture_str = "yeah"
            return gesture_str


class DynamicGestureApp(AIBase):
    """
    动态手势识别应用类 / Dynamic Gesture Recognition Application Class
    继承自AIBase基类 / Inherits from AIBase class
    """
    def __init__(self, kmodel_path, model_input_size, rgb888p_size=[1920,1080],
                 display_size=[1920,1080], debug_mode=0):
        """
        初始化函数 / Initialization function

        参数 / Parameters:
        kmodel_path: 模型文件路径 / Path to the model file
        model_input_size: 模型输入尺寸 / Model input size
        rgb888p_size: 输入图像尺寸 / Input image size
        display_size: 显示尺寸 / Display size
        debug_mode: 调试模式 / Debug mode
        """
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)

        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size

        # 确保宽度16字节对齐 / Ensure width is aligned to 16 bytes
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.debug_mode = debug_mode

        # 初始化两个AI2D处理器,分别用于resize和crop操作
        # Initialize two AI2D processors for resize and crop operations
        self.ai2d_resize = Ai2d(debug_mode)
        self.ai2d_resize.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,
                                      nn.ai2d_format.NCHW_FMT,
                                      np.uint8, np.uint8)

        self.ai2d_crop = Ai2d(debug_mode)
        self.ai2d_crop.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,
                                    nn.ai2d_format.NCHW_FMT,
                                    np.uint8, np.uint8)

        # 初始化模型输入tensor列表 / Initialize model input tensor list
        self.input_tensors = []

        # 定义模型的11个输入tensor的shape / Define shapes of 11 input tensors
        self.gesture_kmodel_input_shape = [
            [1, 3, 224, 224],  # 主要输入 / Main input
            [1,3,56,56],       # 特征图1 / Feature map 1
            [1,4,28,28],       # 特征图2 / Feature map 2
            [1,4,28,28],       # 特征图3 / Feature map 3
            [1,8,14,14],       # 特征图4 / Feature map 4
            [1,8,14,14],       # 特征图5 / Feature map 5
            [1,8,14,14],       # 特征图6 / Feature map 6
            [1,12,14,14],      # 特征图7 / Feature map 7
            [1,12,14,14],      # 特征图8 / Feature map 8
            [1,20,7,7],        # 特征图9 / Feature map 9
            [1,20,7,7]         # 特征图10 / Feature map 10
        ]

        # 预处理参数设置 / Preprocessing parameter settings
        self.resize_shape = 256  # resize尺寸 / Resize dimension
        # 归一化参数 / Normalization parameters
        self.mean_values = np.array([0.485, 0.456, 0.406]).reshape((3,1,1))
        self.std_values = np.array([0.229, 0.224, 0.225]).reshape((3,1,1))

        self.first_data = None  # 首帧数据 / First frame data
        self.max_hist_len = 20  # 历史记录最大长度 / Maximum history length
        self.crop_params = self.get_crop_param()  # 获取裁剪参数 / Get crop parameters

    def config_preprocess(self, input_image_size=None):
        """
        配置预处理流程 / Configure preprocessing pipeline

        参数 / Parameter:
        input_image_size: 输入图像尺寸 / Input image size
        """
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size

            # 配置resize和crop操作 / Configure resize and crop operations
            self.ai2d_resize.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d_resize.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                                 [1,3,self.crop_params[1],self.crop_params[0]])

            self.ai2d_crop.crop(self.crop_params[2],self.crop_params[3],
                               self.crop_params[4],self.crop_params[5])
            self.ai2d_crop.build([1,3,self.crop_params[1],self.crop_params[0]],
                                [1,3,self.model_input_size[1],self.model_input_size[0]])

            # 初始化模型输入tensor / Initialize model input tensors
            inputs_num = self.get_kmodel_inputs_num()
            self.first_data = np.ones(self.gesture_kmodel_input_shape[0], dtype=np.float)
            for i in range(inputs_num):
                data = np.zeros(self.gesture_kmodel_input_shape[i], dtype=np.float)
                self.input_tensors.append(nn.from_numpy(data))

    def preprocess(self, input_np):
        """
        图像预处理流程 / Image preprocessing pipeline

        参数 / Parameter:
        input_np: 输入图像数组 / Input image array
        """
        # 执行resize和crop操作 / Perform resize and crop operations
        resize_tensor = self.ai2d_resize.run(input_np)
        crop_output_tensor = self.ai2d_crop.run(resize_tensor.to_numpy())
        ai2d_output = crop_output_tensor.to_numpy()

        # 数据归一化 / Data normalization
        self.first_data[0] = ai2d_output[0].copy()
        self.first_data[0] = (self.first_data[0]*1.0/255 - self.mean_values)/self.std_values
        self.input_tensors[0] = nn.from_numpy(self.first_data)
        return

    def run(self, input_np, his_logit, history):
        """
        执行推理流程 / Run inference pipeline

        参数 / Parameters:
        input_np: 输入图像数组 / Input image array
        his_logit: 历史logit值 / Historical logit values
        history: 历史预测记录 / Historical predictions

        返回 / Returns:
        idx: 预测的手势类别 / Predicted gesture class
        avg_logit: 平均logit值 / Average logit value
        """
        self.preprocess(input_np)
        outputs = self.inference(self.input_tensors)

        # 更新下一帧输入 / Update next frame input
        outputs_num = self.get_kmodel_outputs_num()
        for i in range(1, outputs_num):
            self.input_tensors[i] = nn.from_numpy(outputs[i])

        return self.postprocess(outputs, his_logit, history)

    def postprocess(self, results, his_logit, history):
        """
        后处理流程 / Post-processing pipeline

        参数 / Parameters:
        results: 模型输出结果 / Model output results
        his_logit: 历史logit值 / Historical logit values
        history: 历史预测记录 / Historical predictions
        """
        with ScopedTiming("postprocess", self.debug_mode > 0):
            his_logit.append(results[0])
            avg_logit = sum(np.array(his_logit))
            idx_ = np.argmax(avg_logit)
            idx = self.gesture_process_output(idx_, history)

            # 如果预测发生变化,重置历史记录 / Reset history if prediction changes
            if (idx_ != idx):
                his_logit_last = his_logit[-1]
                his_logit = []
                his_logit.append(his_logit_last)
            return idx, avg_logit

    def gesture_process_output(self, pred, history):
        """
        手势识别结果处理 / Process gesture recognition results

        参数 / Parameters:
        pred: 当前预测结果 / Current prediction
        history: 历史预测记录 / Historical predictions

        返回 / Returns:
        处理后的预测结果 / Processed prediction
        """
        # 处理特定手势类别 / Handle specific gesture classes
        if (pred == 7 or pred == 8 or pred == 21 or pred == 22 or pred == 3):
            pred = history[-1]
        if (pred == 0 or pred == 4 or pred == 6 or pred == 9 or pred == 14
            or pred == 1 or pred == 19 or pred == 20 or pred == 23 or pred == 24):
            pred = history[-1]
        if (pred == 0):
            pred = 2

        # 处理预测变化 / Handle prediction changes
        if (pred != history[-1]):
            if (len(history) >= 2):
                if (history[-1] != history[len(history)-2]):
                    pred = history[-1]

        # 更新历史记录 / Update history
        history.append(pred)
        if (len(history) > self.max_hist_len):
            history = history[-self.max_hist_len:]
        return history[-1]

    def get_crop_param(self):
        """
        计算裁剪参数 / Calculate crop parameters

        返回 / Returns:
        new_w: 新宽度 / New width
        new_h: 新高度 / New height
        left: 左边界 / Left boundary
        top: 上边界 / Top boundary
        width: 裁剪宽度 / Crop width
        height: 裁剪高度 / Crop height
        """
        ori_w = self.rgb888p_size[0]
        ori_h = self.rgb888p_size[1]
        width = self.model_input_size[0]
        height = self.model_input_size[1]

        # 计算缩放比例 / Calculate scaling ratio
        ratiow = float(self.resize_shape) / ori_w
        ratioh = float(self.resize_shape) / ori_h
        ratio = ratioh if ratiow < ratioh else ratiow

        # 计算新尺寸和裁剪位置 / Calculate new dimensions and crop position
        new_w = int(ratio * ori_w)
        new_h = int(ratio * ori_h)
        top = int((new_h-height)/2)
        left = int((new_w-width)/2)

        return new_w, new_h, left, top, width, height

    def deinit(self):
        """
        资源释放函数 / Resource release function
        释放AI处理器和内存资源 / Release AI processors and memory resources
        """
        with ScopedTiming("deinit", self.debug_mode > 0):
            del self.kpu
            del self.ai2d_resize
            del self.ai2d_crop
            self.tensors.clear()
            del self.tensors
            gc.collect()
            nn.shrink_memory_pool()
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            time.sleep_ms(100)

class DynamicGesture:

    """
    动态手势识别类 / Dynamic Gesture Recognition Class
    实现手掌检测、关键点检测和动态手势识别的完整流程
    Implements complete pipeline for hand detection, keypoint detection and dynamic gesture recognition
    """

    def __init__(self, hand_det_kmodel, hand_kp_kmodel, gesture_kmodel,
                 det_input_size, kp_input_size, gesture_input_size,
                 labels, anchors, confidence_threshold=0.25, nms_threshold=0.3,
                 nms_option=False, strides=[8,16,32], rgb888p_size=[1280,720],
                 display_size=[1920,1080], debug_mode=0):
        """
        初始化函数 / Initialization function

        参数 / Parameters:
        hand_det_kmodel: 手掌检测模型路径 / Path to hand detection model
        hand_kp_kmodel: 手掌关键点模型路径 / Path to hand keypoint model
        gesture_kmodel: 动态手势识别模型路径 / Path to dynamic gesture recognition model
        det_input_size: 检测模型输入尺寸 / Detection model input size
        kp_input_size: 关键点模型输入尺寸 / Keypoint model input size
        gesture_input_size: 手势识别模型输入尺寸 / Gesture model input size
        labels: 标签列表 / Label list
        anchors: 锚框设置 / Anchor settings
        confidence_threshold: 置信度阈值 / Confidence threshold
        nms_threshold: NMS阈值 / NMS threshold
        nms_option: NMS选项 / NMS option
        strides: 特征图步长 / Feature map strides
        rgb888p_size: 输入图像尺寸 / Input image size
        display_size: 显示尺寸 / Display size
        debug_mode: 调试模式 / Debug mode
        """

        # 存储模型路径 / Store model paths
        self.hand_det_kmodel = hand_det_kmodel
        self.hand_kp_kmodel = hand_kp_kmodel
        self.gesture_kmodel = gesture_kmodel

        # 存储模型输入尺寸 / Store model input sizes
        self.det_input_size = det_input_size
        self.kp_input_size = kp_input_size
        self.gesture_input_size = gesture_input_size

        self.labels = labels
        self.anchors = anchors
        self.tx_flag = False

        # 存储阈值参数 / Store threshold parameters
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.nms_option = nms_option
        self.strides = strides

        # 确保宽度16字节对齐 / Ensure width is aligned to 16 bytes
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]

        # 加载状态图标 / Load status icons
        self.bin_width = 150  # 图标宽度 / Icon width
        self.bin_height = 216 # 图标高度 / Icon height

        # 加载四个方向的图标 / Load directional icons
        shang_argb = np.fromfile("/sdcard/utils/shang.bin", dtype=np.uint8)
        self.shang_argb = shang_argb.reshape((self.bin_height, self.bin_width, 4))
        xia_argb = np.fromfile("/sdcard/utils/xia.bin", dtype=np.uint8)
        self.xia_argb = xia_argb.reshape((self.bin_height, self.bin_width, 4))
        zuo_argb = np.fromfile("/sdcard/utils/zuo.bin", dtype=np.uint8)
        self.zuo_argb = zuo_argb.reshape((self.bin_width, self.bin_height, 4))
        you_argb = np.fromfile("/sdcard/utils/you.bin", dtype=np.uint8)
        self.you_argb = you_argb.reshape((self.bin_width, self.bin_height, 4))

        # 定义状态常量 / Define state constants
        self.TRIGGER = 0  # 触发状态 / Trigger state
        self.MIDDLE = 1   # 中间状态 / Middle state
        self.UP = 2       # 向上状态 / Up state
        self.DOWN = 3     # 向下状态 / Down state
        self.LEFT = 4     # 向左状态 / Left state
        self.RIGHT = 5    # 向右状态 / Right state

        self.max_hist_len = 20  # 历史记录最大长度 / Maximum history length
        self.debug_mode = debug_mode

        # 初始化状态变量 / Initialize state variables
        self.cur_state = self.TRIGGER  # 当前状态 / Current state
        self.pre_state = self.TRIGGER  # 前一状态 / Previous state
        self.draw_state = self.TRIGGER # 绘制状态 / Drawing state

        # 初始化各类缓存列表 / Initialize cache lists
        self.vec_flag = []     # 向量标志列表 / Vector flag list
        self.his_logit = []    # 历史logit值列表 / Historical logit list
        self.history = [2]     # 历史预测列表 / Historical prediction list

        # 初始化时间戳 / Initialize timestamps
        self.s_start = time.time_ns()
        self.m_start = None

        # 初始化检测器实例 / Initialize detector instances
        self.hand_det = HandDetApp(
            self.hand_det_kmodel, self.labels,
            model_input_size=self.det_input_size,
            anchors=self.anchors,
            confidence_threshold=self.confidence_threshold,
            nms_threshold=self.nms_threshold,
            nms_option=self.nms_option,
            strides=self.strides,
            rgb888p_size=self.rgb888p_size,
            display_size=self.display_size,
            debug_mode=0
        )

        self.hand_kp = HandKPClassApp(
            self.hand_kp_kmodel,
            model_input_size=self.kp_input_size,
            rgb888p_size=self.rgb888p_size,
            display_size=self.display_size
        )

        self.dg = DynamicGestureApp(
            self.gesture_kmodel,
            model_input_size=self.gesture_input_size,
            rgb888p_size=self.rgb888p_size,
            display_size=self.display_size
        )

        # 配置预处理 / Configure preprocessing
        self.hand_det.config_preprocess()
        self.dg.config_preprocess()

    def run(self, input_np):

        """
        运行手势识别 / Run gesture recognition

        参数 / Parameter:
        input_np: 输入图像 / Input image

        返回 / Returns:
        根据当前状态返回检测结果或识别结果 / Returns detection or recognition results based on current state
        """
        if self.cur_state == self.TRIGGER:
            # 手掌检测模式 / Hand detection mode
            det_boxes = self.hand_det.run(input_np)
            boxes = []
            gesture_res = []

            for det_box in det_boxes:
                # 获取检测框坐标 / Get detection box coordinates
                x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]
                w, h = int(x2 - x1), int(y2 - y1)

                # 过滤无效检测框 / Filter invalid detection boxes
                if (h < (0.1 * self.rgb888p_size[1])):
                    continue
                if (w < (0.25 * self.rgb888p_size[0]) and
                    ((x1 < (0.03 * self.rgb888p_size[0])) or
                     (x2 > (0.97 * self.rgb888p_size[0])))):
                    continue
                if (w < (0.15 * self.rgb888p_size[0]) and
                    ((x1 < (0.01 * self.rgb888p_size[0])) or
                     (x2 > (0.99 * self.rgb888p_size[0])))):
                    continue

                # 执行关键点检测 / Perform keypoint detection
                self.hand_kp.config_preprocess(det_box)
                hk_results, gesture_str = self.hand_kp.run(input_np)
                boxes.append(det_box)
                gesture_res.append((hk_results, gesture_str))

            return boxes, gesture_res
        else:
            # 动态手势识别模式 / Dynamic gesture recognition mode
            idx, avg_logit = self.dg.run(input_np, self.his_logit, self.history)
            return idx, avg_logit

    def draw_result(self, pl, output1, output2):
        """
        绘制识别结果 / Draw recognition results

        参数 / Parameters:
        pl: 绘图层对象 / Drawing layer object
        output1: 第一阶段输出(检测框) / First stage output (detection boxes)
        output2: 第二阶段输出(关键点和手势) / Second stage output (keypoints and gestures)
        """
        # 清空绘图缓存 / Clear drawing cache
        pl.osd_img.clear()
        draw_img_np = np.zeros((self.display_size[1], self.display_size[0], 4), dtype=np.uint8)
        draw_img = image.Image(self.display_size[0], self.display_size[1],
                            image.ARGB8888, alloc=image.ALLOC_REF, data=draw_img_np)

        if self.cur_state == self.TRIGGER:
            # 触发状态下的处理 / Processing in trigger state
            for i in range(len(output1)):
                hk_results, gesture = output2[i][0], output2[i][1]

                # 检测到"five"或"yeah"手势 / Detect "five" or "yeah" gesture
                if ((gesture == "five") or (gesture == "yeah")):
                    # 计算手势方向向量 / Calculate gesture direction vector
                    v_x = hk_results[24] - hk_results[0]
                    v_y = hk_results[25] - hk_results[1]
                    angle = self.hand_kp.hk_vector_2d_angle([v_x,v_y], [1.0,0.0])
                    if (v_y > 0):
                        angle = 360 - angle

                    # 根据角度判断手势方向 / Determine gesture direction based on angle
                    if ((70.0 <= angle) and (angle < 110.0)):  # 向上 / Upward
                        if ((self.pre_state != self.UP) or (self.pre_state != self.MIDDLE)):
                            self.vec_flag.append(self.pre_state)
                        if ((len(self.vec_flag) > 10) or (self.pre_state == self.UP) or
                            (self.pre_state == self.MIDDLE) or (self.pre_state == self.TRIGGER)):
                            draw_img_np[:self.bin_height,:self.bin_width,:] = self.shang_argb
                            self.cur_state = self.UP

                    elif ((110.0 <= angle) and (angle < 225.0)):  # 向右 / Rightward
                        if (self.pre_state != self.RIGHT):
                            self.vec_flag.append(self.pre_state)
                        if ((len(self.vec_flag) > 10) or (self.pre_state == self.RIGHT) or
                            (self.pre_state == self.TRIGGER)):
                            draw_img_np[:self.bin_width,:self.bin_height,:] = self.you_argb
                            self.cur_state = self.RIGHT

                    elif((225.0 <= angle) and (angle < 315.0)):  # 向下 / Downward
                        if (self.pre_state != self.DOWN):
                            self.vec_flag.append(self.pre_state)
                        if ((len(self.vec_flag) > 10) or (self.pre_state == self.DOWN) or
                            (self.pre_state == self.TRIGGER)):
                            draw_img_np[:self.bin_height,:self.bin_width,:] = self.xia_argb
                            self.cur_state = self.DOWN

                    else:  # 向左 / Leftward
                        if (self.pre_state != self.LEFT):
                            self.vec_flag.append(self.pre_state)
                        if ((len(self.vec_flag) > 10) or (self.pre_state == self.LEFT) or
                            (self.pre_state == self.TRIGGER)):
                            draw_img_np[:self.bin_width,:self.bin_height,:] = self.zuo_argb
                            self.cur_state = self.LEFT

                    self.m_start = time.time_ns()
                self.his_logit = []

        else:
            # 非触发状态下的处理 / Processing in non-trigger state
            idx, avg_logit = output1, output2[0]

            # 处理不同状态下的手势动作 / Handle gesture actions in different states
            if (self.cur_state == self.UP):
                draw_img_np[:self.bin_height,:self.bin_width,:] = self.shang_argb
                if ((idx == 15) or (idx == 10)):  # 向上挥动确认 / Upward wave confirmation
                    self.vec_flag.clear()
                    if (((avg_logit[idx] >= 0.7) and (len(self.his_logit) >= 2)) or
                        ((avg_logit[idx] >= 0.3) and (len(self.his_logit) >= 4))):
                        self.s_start = time.time_ns()
                        self.cur_state = self.TRIGGER
                        self.draw_state = self.DOWN
                        self.history = [2]
                    self.pre_state = self.UP
                elif ((idx == 25) or (idx == 26)):  # 中间位置确认 / Middle position confirmation
                    self.vec_flag.clear()
                    if (((avg_logit[idx] >= 0.4) and (len(self.his_logit) >= 2)) or
                        ((avg_logit[idx] >= 0.3) and (len(self.his_logit) >= 3))):
                        self.s_start = time.time_ns()
                        self.cur_state = self.TRIGGER
                        self.draw_state = self.MIDDLE
                        self.history = [2]
                    self.pre_state = self.MIDDLE
                else:
                    self.his_logit.clear()

            # 处理其他方向的状态(RIGHT/DOWN/LEFT)的逻辑类似
            # Similar logic for other directional states (RIGHT/DOWN/LEFT)
            elif (self.cur_state == self.RIGHT):
                draw_img_np[:self.bin_width,:self.bin_height,:] = self.you_argb
                if  ((idx==16)or(idx==11)) :
                    self.vec_flag.clear()
                    if (((avg_logit[idx] >= 0.4) and (len(self.his_logit) >= 2)) or ((avg_logit[idx] >= 0.3) and (len(self.his_logit) >= 3))):
                        self.s_start = time.time_ns()
                        self.cur_state = self.TRIGGER
                        self.draw_state = self.RIGHT
                        self.history = [2]
                    self.pre_state = self.RIGHT
                else:
                    self.his_logit.clear()
            elif (self.cur_state == self.DOWN):
                draw_img_np[:self.bin_height,:self.bin_width,:] = self.xia_argb
                if  ((idx==18)or(idx==13)):
                    self.vec_flag.clear()
                    if (((avg_logit[idx] >= 0.4) and (len(self.his_logit) >= 2)) or ((avg_logit[idx] >= 0.3) and (len(self.his_logit) >= 3))):
                        self.s_start = time.time_ns()
                        self.cur_state = self.TRIGGER
                        self.draw_state = self.UP
                        self.history = [2]
                    self.pre_state = self.DOWN
                else:
                    self.his_logit.clear()
            elif (self.cur_state == self.LEFT):
                draw_img_np[:self.bin_width,:self.bin_height,:] = self.zuo_argb
                if ((idx==17)or(idx==12)):
                    self.vec_flag.clear()
                    if (((avg_logit[idx] >= 0.4) and (len(self.his_logit) >= 2)) or ((avg_logit[idx] >= 0.3) and (len(self.his_logit) >= 3))):
                        self.s_start = time.time_ns()
                        self.cur_state = self.TRIGGER
                        self.draw_state = self.LEFT
                        self.history = [2]
                    self.pre_state = self.LEFT
                else:
                    self.his_logit.clear()

            self.elapsed_time = round((time.time_ns() - self.m_start)/1000000)

            # 超时处理 / Timeout handling
            if ((self.cur_state != self.TRIGGER) and (self.elapsed_time > 2000)):
                self.cur_state = self.TRIGGER
                self.pre_state = self.TRIGGER

        # 绘制结果显示 / Draw result display
        self.elapsed_ms_show = round((time.time_ns()-self.s_start)/1000000)
        if (self.elapsed_ms_show < 1000):
            gesture_dir = ""
            # 根据不同状态绘制不同的箭头和文字 / Draw different arrows and text based on state
            if (self.draw_state == self.UP):
                draw_img.draw_arrow(self.display_size[0]//2, self.display_size[1]//2,
                                self.display_size[0]//2, self.display_size[1]//2-100,
                                (155,170,190,230), thickness=13)
                draw_img.draw_string_advanced(self.display_size[0]//2-50,
                                            self.display_size[1]//2-50, 32, "向上 UP")
                gesture_dir = "UP"
            elif (self.draw_state == self.LEFT):
                draw_img.draw_arrow(self.display_size[0]//2, self.display_size[1]//2,
                                self.display_size[0]//2-100, self.display_size[1]//2,
                                (155,170,190,230), thickness=13)
                draw_img.draw_string_advanced(self.display_size[0]//2-50,
                                            self.display_size[1]//2-50, 32, "向左 LEFT")
                gesture_dir = "LEFT"
            elif (self.draw_state == self.DOWN):
                draw_img.draw_arrow(self.display_size[0]//2,self.display_size[1]//2,self.display_size[0]//2,self.display_size[1]//2+100, (155,170,190,230), thickness=13)                             # 判断为向下挥动时，画一个向下的箭头
                draw_img.draw_string_advanced(self.display_size[0]//2-50,self.display_size[1]//2-50,32,"向下 DOWN")
                gesture_dir = "DOWN"
            elif (self.draw_state == self.RIGHT):
                draw_img.draw_arrow(self.display_size[0]//2,self.display_size[1]//2,self.display_size[0]//2+100,self.display_size[1]//2, (155,170,190,230), thickness=13)                               # 判断为向左挥动时，画一个向左的箭头
                draw_img.draw_string_advanced(self.display_size[0]//2-50,self.display_size[1]//2-50,32,"向右 RIGHT")
                gesture_dir = "RIGHT"
            elif (self.draw_state == self.MIDDLE):
                draw_img.draw_circle(self.display_size[0]//2,self.display_size[1]//2,100, (255,170,190,230), thickness=2, fill=True)                       # 判断为五指捏合手势时，画一个实心圆
                draw_img.draw_string_advanced(self.display_size[0]//2-50,self.display_size[1]//2-50,32,"中间 MIDDLE")
                gesture_dir = "MIDDLE"
            if len(gesture_dir) > 0 and self.tx_flag == False:
                self.tx_flag = True
                pto_data = pto.get_hand_gesture_data(gesture_dir)
                uart.send(pto_data)
                print(pto_data)

        else:
            self.draw_state = self.TRIGGER
            self.tx_flag = False

        # 更新显示 / Update display
        pl.osd_img.copy_from(draw_img)


def exce_demo(pl):

    print("模型加载中 ...")

    screen_width = 640
    screen_height = 480

    banner_width = screen_width
    banner_height = 100

    banner_x = (screen_width - banner_width) // 2
    banner_y = (screen_height - banner_height) // 2

    pl.osd_img.draw_rectangle(banner_x, banner_y, banner_width, banner_height, color=(150, 0, 165, 253), fill=True)

    text = "模型加载中 | Model loading ..."
    text_x = (screen_width - len(text) * 8) // 2  # 8 是字符宽度
    text_y = banner_y + (banner_height - 10) // 2  # 10 是字符高度

    pl.osd_img.draw_string_advanced(text_x, text_y,18, text, color=(255, 255, 255))
    pl.show_image()

    global dg

    display_mode = pl.display_mode
    rgb888p_size = pl.rgb888p_size
    display_size=pl.display_size

    hand_det_kmodel_path="/sdcard/kmodel/hand_det.kmodel"
    hand_kp_kmodel_path="/sdcard/kmodel/handkp_det.kmodel"
    gesture_kmodel_path="/sdcard/kmodel/gesture.kmodel"

    hand_det_input_size=[512,512]
    hand_kp_input_size=[256,256]
    gesture_input_size=[224,224]
    confidence_threshold=0.2
    nms_threshold=0.5
    labels=["hand"]
    anchors = [26,27, 53,52, 75,71, 80,99, 106,82, 99,134, 140,113, 161,172, 245,276]

    try:
        dg=DynamicGesture(hand_det_kmodel_path,hand_kp_kmodel_path,gesture_kmodel_path,det_input_size=hand_det_input_size,kp_input_size=hand_kp_input_size,gesture_input_size=gesture_input_size,labels=labels,anchors=anchors,confidence_threshold=confidence_threshold,nms_threshold=nms_threshold,nms_option=False,strides=[8,16,32],rgb888p_size=rgb888p_size,display_size=display_size)

        print("模型加载成功，手势识别开始 / Model init finish")
        while True:
            img=pl.get_frame()
            output1,output2=dg.run(img)
            dg.draw_result(pl,output1,output2)
            pl.show_image()
            gc.collect()
    except Exception as e:
        print("动态手势识别功能退出 / exit")
    finally:
        exit_demo()

def exit_demo():
    global dg
    dg.hand_det.deinit()
    dg.hand_kp.deinit()
    dg.dg.deinit()


if __name__=="__main__":

    rgb888p_size=[640,360]
    display_size=[640,480]
    display_mode="lcd"

    pl=PipeLine(rgb888p_size=rgb888p_size,display_size=display_size,display_mode=display_mode)
    pl.create(hmirror=True)

    exce_demo(pl)
