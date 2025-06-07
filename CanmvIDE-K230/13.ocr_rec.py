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

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()

# 自定义OCR检测类
# Custom OCR detection class
class OCRDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, mask_threshold=0.5, box_threshold=0.2, rgb888p_size=[224,224], display_size=[1920,1080], debug_mode=0):
        """
        初始化OCR检测应用
        Initialize the OCR detection application

        参数:
        kmodel_path: 模型路径 / Model path
        model_input_size: 模型输入大小 / Model input size
        mask_threshold: 掩码阈值，用于区分前景和背景 / Mask threshold for foreground/background separation
        box_threshold: 边界框阈值，用于确定检测结果 / Box threshold for detection results
        rgb888p_size: 输入图像大小 / Input image size
        display_size: 显示大小 / Display size
        debug_mode: 调试模式级别 / Debug mode level
        """
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        # 模型输入分辨率 / Model input resolution
        self.model_input_size = model_input_size
        # 分类阈值 / Classification thresholds
        self.mask_threshold = mask_threshold  # 掩码阈值 / Mask threshold
        self.box_threshold = box_threshold    # 框阈值 / Box threshold
        # sensor给到AI的图像分辨率，宽度16字节对齐 / Image resolution from sensor to AI, width aligned to 16 bytes
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        # 显示分辨率 / Display resolution
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        # Ai2d实例，用于实现模型预处理 / Ai2d instance for model preprocessing
        self.ai2d = Ai2d(debug_mode)
        # 设置Ai2d的输入输出格式和类型 / Set input and output formats and types for Ai2d
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    # 配置预处理操作，这里使用了pad和resize
    # Configure preprocessing operations, using pad and resize here
    # Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/app/libs/AI2D.py查看
    # Ai2d supports crop/shift/pad/resize/affine, see /sdcard/app/libs/AI2D.py for details
    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，您可以通过设置input_image_size自行修改输入尺寸
            # Initialize ai2d preprocessing config, default is the size from sensor to AI
            # You can modify the input size by setting input_image_size
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right = self.get_padding_param()
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [0, 0, 0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]], [1, 3, self.model_input_size[1], self.model_input_size[0]])

    # 自定义当前任务的后处理
    # Custom post-processing for the current task
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            # chw2hwc 将数据从[channel,height,width]转换为[height,width,channel]格式
            # Convert data from [channel,height,width] to [height,width,channel] format
            hwc_array = self.chw2hwc(self.cur_img)
            # 这里使用了aicube封装的接口ocr_post_process做后处理
            # Using aicube's ocr_post_process interface for post-processing
            # 返回的det_boxes结构为[[crop_array_nhwc,[p1_x,p1_y,p2_x,p2_y,p3_x,p3_y,p4_x,p4_y]],...]
            # The structure of returned det_boxes is [[crop_array_nhwc,[p1_x,p1_y,p2_x,p2_y,p3_x,p3_y,p4_x,p4_y]],...]
            det_boxes = aicube.ocr_post_process(results[0][:,:,:,0].reshape(-1),
                                               hwc_array.reshape(-1),
                                               self.model_input_size,
                                               self.rgb888p_size,
                                               self.mask_threshold,
                                               self.box_threshold)
            return det_boxes

    # 计算padding参数
    # Calculate padding parameters
    def get_padding_param(self):
        # 右padding或下padding / Right padding or bottom padding
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]
        input_width = self.rgb888p_size[0]
        input_high = self.rgb888p_size[1]
        # 计算缩放比例 / Calculate scaling ratios
        ratio_w = dst_w / input_width
        ratio_h = dst_h / input_high
        # 选择较小的缩放比例，保持原始宽高比 / Choose the smaller ratio to maintain the original aspect ratio
        if ratio_w < ratio_h:
            ratio = ratio_w
        else:
            ratio = ratio_h
        # 计算缩放后的新尺寸 / Calculate new dimensions after scaling
        new_w = (int)(ratio * input_width)
        new_h = (int)(ratio * input_high)
        # 计算padding量 / Calculate padding amounts
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        # 四边的padding值 / Padding values for all four sides
        top = (int)(round(0))
        bottom = (int)(round(dh * 2 + 0.1))
        left = (int)(round(0))
        right = (int)(round(dw * 2 - 0.1))
        return top, bottom, left, right

    # chw2hwc 转换通道顺序
    # chw2hwc converts channel order
    def chw2hwc(self, features):
        # 从[channel,height,width]转换为[height,width,channel]
        # Convert from [channel,height,width] to [height,width,channel]
        ori_shape = (features.shape[0], features.shape[1], features.shape[2])
        c_hw_ = features.reshape((ori_shape[0], ori_shape[1] * ori_shape[2]))
        hw_c_ = c_hw_.transpose()
        new_array = hw_c_.copy()
        hwc_array = new_array.reshape((ori_shape[1], ori_shape[2], ori_shape[0]))
        # 释放中间变量，降低内存占用 / Release intermediate variables to reduce memory usage
        del c_hw_
        del hw_c_
        del new_array
        return hwc_array

# 自定义OCR识别任务类
# Custom OCR recognition task class
class OCRRecognitionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, dict_path, rgb888p_size=[1920,1080], display_size=[1920,1080], debug_mode=0):
        """
        初始化OCR识别应用
        Initialize OCR recognition application

        参数:
        kmodel_path: 模型路径 / Model path
        model_input_size: 模型输入大小 / Model input size
        dict_path: 字典文件路径，用于将识别结果映射为文字 / Dictionary file path for mapping recognition results to text
        rgb888p_size: 输入图像大小 / Input image size
        display_size: 显示大小 / Display size
        debug_mode: 调试模式级别 / Debug mode level
        """
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        # kmodel路径 / kmodel path
        self.kmodel_path = kmodel_path
        # 识别模型输入分辨率 / Recognition model input resolution
        self.model_input_size = model_input_size
        self.dict_path = dict_path
        # sensor给到AI的图像分辨率，宽16字节对齐 / Image resolution from sensor to AI, width aligned to 16 bytes
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        # 视频输出VO分辨率，宽16字节对齐 / Video output VO resolution, width aligned to 16 bytes
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        # debug模式 / debug mode
        self.debug_mode = debug_mode
        self.dict_word = None
        # 读取OCR的字典 / Read OCR dictionary
        self.read_dict()
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.RGB_packed, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    # 配置预处理操作
    # Configure preprocessing operations
    def config_preprocess(self, input_image_size=None, input_np=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right = self.get_padding_param(ai2d_input_size, self.model_input_size)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [0, 0, 0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            # 如果传入input_np，输入shape为input_np的shape,如果不传入，输入shape为[1,3,ai2d_input_size[1],ai2d_input_size[0]]
            # If input_np is provided, use its shape as input shape
            # Otherwise, use [1,3,ai2d_input_size[1],ai2d_input_size[0]] as input shape
            self.ai2d.build([input_np.shape[0], input_np.shape[1], input_np.shape[2], input_np.shape[3]],
                           [1, 3, self.model_input_size[1], self.model_input_size[0]])

    # 自定义后处理，results是模型输出的array列表
    # Custom post-processing, results is the array list output by the model
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            # 获取每一列的最高概率对应的字符索引 / Get character index corresponding to highest probability for each column
            preds = np.argmax(results[0], axis=2).reshape((-1))
            output_txt = ""
            for i in range(len(preds)):
                # 当前识别字符不是字典的最后一个字符并且和前一个字符不重复（去重），加入识别结果字符串
                # Add character to result string if it's not the last character in dictionary
                # and not a duplicate of the previous character (deduplication)
                if preds[i] != (len(self.dict_word) - 1) and (not (i > 0 and preds[i - 1] == preds[i])):
                    output_txt = output_txt + self.dict_word[preds[i]]
            return output_txt

    # 计算padding参数
    # Calculate padding parameters
    def get_padding_param(self, src_size, dst_size):
        # 右padding或下padding / Right or bottom padding
        dst_w = dst_size[0]
        dst_h = dst_size[1]
        input_width = src_size[0]
        input_high = src_size[1]
        # 计算缩放比例 / Calculate scaling ratios
        ratio_w = dst_w / input_width
        ratio_h = dst_h / input_high
        # 选择较小的缩放比例，保持原始宽高比 / Choose smaller ratio to maintain aspect ratio
        if ratio_w < ratio_h:
            ratio = ratio_w
        else:
            ratio = ratio_h
        # 计算缩放后的新尺寸 / Calculate new dimensions after scaling
        new_w = (int)(ratio * input_width)
        new_h = (int)(ratio * input_high)
        # 计算padding量 / Calculate padding amounts
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        # 四边的padding值 / Padding values for all four sides
        top = (int)(round(0))
        bottom = (int)(round(dh * 2 + 0.1))
        left = (int)(round(0))
        right = (int)(round(dw * 2 - 0.1))
        return top, bottom, left, right

    # 读取字典文件
    # Read dictionary file
    def read_dict(self):
        if self.dict_path != "":
            with open(dict_path, 'r') as file:
                line_one = file.read(100000)
                line_list = line_one.split("\r\n")
            # 创建字典，将索引映射到字符 / Create dictionary mapping indices to characters
            self.dict_word = {num: char.replace("\r", "").replace("\n", "") for num, char in enumerate(line_list)}


# OCR检测和识别的组合类
# Combined class for OCR detection and recognition
class OCRDetRec:
    def __init__(self, ocr_det_kmodel, ocr_rec_kmodel, det_input_size, rec_input_size, dict_path,
                mask_threshold=0.25, box_threshold=0.3, rgb888p_size=[1920,1080], display_size=[1920,1080], debug_mode=0):
        """
        初始化OCR检测和识别的组合应用
        Initialize combined OCR detection and recognition application

        参数:
        ocr_det_kmodel: OCR检测模型路径 / OCR detection model path
        ocr_rec_kmodel: OCR识别模型路径 / OCR recognition model path
        det_input_size: 检测模型输入大小 / Detection model input size
        rec_input_size: 识别模型输入大小 / Recognition model input size
        dict_path: 字典文件路径 / Dictionary file path
        mask_threshold: 掩码阈值 / Mask threshold
        box_threshold: 框阈值 / Box threshold
        rgb888p_size: 输入图像大小 / Input image size
        display_size: 显示大小 / Display size
        debug_mode: 调试模式级别 / Debug mode level
        """
        # OCR检测模型路径 / OCR detection model path
        self.ocr_det_kmodel = ocr_det_kmodel
        # OCR识别模型路径 / OCR recognition model path
        self.ocr_rec_kmodel = ocr_rec_kmodel
        # OCR检测模型输入分辨率 / OCR detection model input resolution
        self.det_input_size = det_input_size
        # OCR识别模型输入分辨率 / OCR recognition model input resolution
        self.rec_input_size = rec_input_size
        # 字典路径 / Dictionary path
        self.dict_path = dict_path
        # 置信度阈值 / Confidence threshold
        self.mask_threshold = mask_threshold
        # nms阈值 / NMS threshold
        self.box_threshold = box_threshold
        # sensor给到AI的图像分辨率，宽16字节对齐 / Image resolution from sensor to AI, width aligned to 16 bytes
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        # 视频输出VO分辨率，宽16字节对齐 / Video output resolution, width aligned to 16 bytes
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        # debug_mode模式 / debug_mode mode
        self.debug_mode = debug_mode
        # 创建检测和识别实例 / Create detection and recognition instances
        self.ocr_det = OCRDetectionApp(self.ocr_det_kmodel, model_input_size=self.det_input_size,
                                     mask_threshold=self.mask_threshold, box_threshold=self.box_threshold,
                                     rgb888p_size=self.rgb888p_size, display_size=self.display_size, debug_mode=0)
        self.ocr_rec = OCRRecognitionApp(self.ocr_rec_kmodel, model_input_size=self.rec_input_size,
                                       dict_path=self.dict_path, rgb888p_size=self.rgb888p_size,
                                       display_size=self.display_size)
        # 配置检测模型预处理 / Configure detection model preprocessing
        self.ocr_det.config_preprocess()

    # run函数，执行推理
    # Run function for inference
    def run(self, input_np):
        # 先进行OCR检测 / First perform OCR detection
        det_res = self.ocr_det.run(input_np)
        boxes = []
        ocr_res = []
        for det in det_res:
            # 对得到的每个检测框执行OCR识别 / Perform OCR recognition on each detected box
            self.ocr_rec.config_preprocess(input_image_size=[det[0].shape[2], det[0].shape[1]], input_np=det[0])
            ocr_str = self.ocr_rec.run(det[0])
            ocr_res.append(ocr_str)
            boxes.append(det[1])
            # 执行垃圾回收，减少内存占用 / Perform garbage collection to reduce memory usage
            gc.collect()
        return boxes, ocr_res

    # 绘制OCR检测识别效果
    # Draw OCR detection and recognition results
    def draw_result(self, pl, det_res, rec_res):
        # 清除叠加层 / Clear overlay layer
        pl.osd_img.clear()
        if det_res:
            # 循环绘制所有检测到的框 / Loop through all detected boxes
            for j in range(len(det_res)):
                # 将原图的坐标点转换成显示的坐标点，循环绘制四条直线，得到一个矩形框
                # Convert coordinates from original image to display coordinates
                # Draw four lines to form a rectangle
                for i in range(4):
                    # 坐标转换 / Coordinate conversion
                    x1 = det_res[j][(i * 2)] / self.rgb888p_size[0] * self.display_size[0]
                    y1 = det_res[j][(i * 2 + 1)] / self.rgb888p_size[1] * self.display_size[1]
                    x2 = det_res[j][((i + 1) * 2) % 8] / self.rgb888p_size[0] * self.display_size[0]
                    y2 = det_res[j][((i + 1) * 2 + 1) % 8] / self.rgb888p_size[1] * self.display_size[1]
                    # 绘制线段 / Draw line segment
                    pl.osd_img.draw_line((int(x1), int(y1), int(x2), int(y2)), color=(255, 0, 0, 255), thickness=5)
                # 在框上方绘制识别文本 / Draw recognized text above the box
                pl.osd_img.draw_string_advanced(int(x1), int(y1), 32, rec_res[j], color=(0, 0, 255))

                pto_data = pto.get_ocr_rec_data(rec_res[j])
                uart.send(pto_data)
                print(pto_data)


if __name__ == "__main__":
    # 显示模式，默认"hdmi"，可以选择"hdmi"和"lcd"，k230d受限内存不支持
    # Display mode, default is "hdmi", can choose between "hdmi" and "lcd", k230d with limited memory doesn't support
    display_mode = "lcd"
    if display_mode == "hdmi":
        display_size = [1920, 1080]
    else:
        display_size = [640, 480]
    # OCR检测模型路径 / OCR detection model path
    ocr_det_kmodel_path = "/sdcard/kmodel/ocr_det_int16.kmodel"
    # OCR识别模型路径 / OCR recognition model path
    ocr_rec_kmodel_path = "/sdcard/kmodel/ocr_rec_int16.kmodel"
    # 其他参数 / Other parameters
    dict_path = "/sdcard/utils/dict.txt"
    rgb888p_size = [640, 360]
    ocr_det_input_size = [640, 640]
    ocr_rec_input_size = [512, 32]
    mask_threshold = 0.25
    box_threshold = 0.3

    # 初始化PipeLine，只关注传给AI的图像分辨率，显示的分辨率
    # Initialize PipeLine, focusing only on image resolution for AI and display
    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode)
    pl.create()
    # 创建OCR检测识别实例 / Create OCR detection and recognition instance
    ocr = OCRDetRec(ocr_det_kmodel_path, ocr_rec_kmodel_path, det_input_size=ocr_det_input_size,
                   rec_input_size=ocr_rec_input_size, dict_path=dict_path, mask_threshold=mask_threshold,
                   box_threshold=box_threshold, rgb888p_size=rgb888p_size, display_size=display_size)
    while True:
        # 计时整个处理流程 / Time the entire processing flow
        with ScopedTiming("total", 0):
            img = pl.get_frame()                  # 获取当前帧 / Get current frame
            det_res, rec_res = ocr.run(img)       # 推理当前帧 / Inference on current frame
            ocr.draw_result(pl, det_res, rec_res) # 绘制当前帧推理结果 / Draw inference results for current frame
            pl.show_image()                       # 展示当前帧推理结果 / Display inference results for current frame
            # 执行垃圾回收，减少内存占用 / Perform garbage collection to reduce memory usage
            gc.collect()
    # 释放资源 / Release resources
    ocr.ocr_det.deinit()
    ocr.ocr_rec.deinit()
    pl.destroy()
