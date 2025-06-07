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
import aidemo
import random
import gc
import sys

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()


lr = None

# 车牌检测类 License Plate Detection Class
class LicenceDetectionApp(AIBase):
    """
    车牌检测应用类，继承自AIBase
    License plate detection application class, inherited from AIBase
    """
    def __init__(self, kmodel_path, model_input_size, confidence_threshold=0.5, nms_threshold=0.2, rgb888p_size=[224,224], display_size=[1920,1080], debug_mode=0):
        """
        初始化函数 Initialization function
        参数 Parameters:
            kmodel_path: 模型路径 Model path
            model_input_size: 模型输入尺寸 Model input size
            confidence_threshold: 置信度阈值 Confidence threshold
            nms_threshold: NMS阈值 NMS threshold
            rgb888p_size: 输入图像尺寸 Input image size
            display_size: 显示尺寸 Display size
            debug_mode: 调试模式 Debug mode
        """
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        # 确保宽度是16的倍数 Ensure width is multiple of 16
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode

        # 初始化AI2D实例用于图像预处理 Initialize AI2D instance for image preprocessing
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        """
        配置图像预处理参数 Configure image preprocessing parameters
        """
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            # 配置双线性插值方法 Configure bilinear interpolation method
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self, results):
        """
        后处理函数 Postprocessing function
        对模型输出结果进行处理 Process model output results
        """
        with ScopedTiming("postprocess", self.debug_mode > 0):
            det_res = aidemo.licence_det_postprocess(results,
                                                   [self.rgb888p_size[1], self.rgb888p_size[0]],
                                                   self.model_input_size,
                                                   self.confidence_threshold,
                                                   self.nms_threshold)
            return det_res

    def draw_result(self, pl, dets):
        """
        绘制检测结果 Draw detection results
        参数 Parameters:
            pl: PipeLine实例 PipeLine instance
            dets: 检测结果 Detection results
        """
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                pl.osd_img.clear()
                point_8 = np.zeros((8), dtype=np.int16)
                for det in dets:
                    # 坐标转换 Coordinate conversion
                    for i in range(4):
                        x = det[i * 2 + 0] / self.rgb888p_size[0] * self.display_size[0]
                        y = det[i * 2 + 1] / self.rgb888p_size[1] * self.display_size[1]
                        point_8[i * 2 + 0] = int(x)
                        point_8[i * 2 + 1] = int(y)
                    # 绘制检测框 Draw detection box
                    for i in range(4):
                        pl.osd_img.draw_line(point_8[i * 2 + 0],
                                           point_8[i * 2 + 1],
                                           point_8[(i + 1) % 4 * 2 + 0],
                                           point_8[(i + 1) % 4 * 2 + 1],
                                           color=(255, 0, 255, 0),
                                           thickness=4)
            else:
                pl.osd_img.clear()


class LicenceRecognitionApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,rgb888p_size=[1920,1080],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        # kmodel路径
        self.kmodel_path=kmodel_path
        # 检测模型输入分辨率
        self.model_input_size=model_input_size
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        # 视频输出VO分辨率，宽16字节对齐
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        # debug模式
        self.debug_mode=debug_mode
        # 车牌字符字典
        self.dict_rec = ["挂", "使", "领", "澳", "港", "皖", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑", "苏", "浙", "京", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤", "桂", "琼", "川", "贵", "云", "藏", "陕", "甘", "青", "宁", "新", "警", "学", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E", "F", "G", "H", "J", "K", "L", "M", "N", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "_", "-"]
        self.dict_size = len(self.dict_rec)
        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)


    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            output_data=results[0].reshape((-1,self.dict_size))
            max_indices = np.argmax(output_data, axis=1)
            result_str = ""
            for i in range(max_indices.shape[0]):
                index = max_indices[i]
                if index > 0 and (i == 0 or index != max_indices[i - 1]):
                    result_str += self.dict_rec[index - 1]
            return result_str

class LicenceRec:
    """
    车牌检测和识别的整合类，包含检测和识别两个功能模块
    Integrated class for license plate detection and recognition, including detection and recognition modules
    """
    def __init__(self, licence_det_kmodel, licence_rec_kmodel, det_input_size, rec_input_size,
                 confidence_threshold=0.25, nms_threshold=0.3, rgb888p_size=[1920,1080],
                 display_size=[1920,1080], debug_mode=0):
        """
        初始化函数
        Initialization function

        参数说明 Parameters:
        licence_det_kmodel: 车牌检测模型路径 Path to license plate detection model
        licence_rec_kmodel: 车牌识别模型路径 Path to license plate recognition model
        det_input_size: 检测模型的输入尺寸 Input size for detection model
        rec_input_size: 识别模型的输入尺寸 Input size for recognition model
        confidence_threshold: 置信度阈值 Confidence threshold for detection
        nms_threshold: 非极大值抑制阈值 Non-maximum suppression threshold
        rgb888p_size: 输入图像尺寸 Input image size
        display_size: 显示尺寸 Display size
        debug_mode: 调试模式 Debug mode
        """
        # 初始化成员变量 Initialize member variables
        self.licence_det_kmodel = licence_det_kmodel
        self.licence_rec_kmodel = licence_rec_kmodel
        self.det_input_size = det_input_size
        self.rec_input_size = rec_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold

        # 确保图像宽度是16的倍数 Ensure image width is multiple of 16
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.debug_mode = debug_mode

        # 初始化检测和识别模型 Initialize detection and recognition models
        self.licence_det = LicenceDetectionApp(
            self.licence_det_kmodel,
            model_input_size=self.det_input_size,
            confidence_threshold=self.confidence_threshold,
            nms_threshold=self.nms_threshold,
            rgb888p_size=self.rgb888p_size,
            display_size=self.display_size,
            debug_mode=0)
        self.licence_rec = LicenceRecognitionApp(
            self.licence_rec_kmodel,
            model_input_size=self.rec_input_size,
            rgb888p_size=self.rgb888p_size)

        # 配置检测模型的预处理 Configure preprocessing for detection model
        self.licence_det.config_preprocess()

    def run(self, input_np):
        """
        执行车牌检测和识别的主要流程
        Main pipeline for license plate detection and recognition

        参数 Parameters:
        input_np: 输入图像(numpy array格式) Input image in numpy array format

        返回 Returns:
        det_boxes: 检测到的车牌位置 Detected license plate positions
        rec_res: 识别的车牌字符 Recognized license plate characters
        """
        # 执行车牌检测 Perform license plate detection
        det_boxes = self.licence_det.run(input_np)

        # 对检测到的区域进行预处理 Preprocess detected regions
        imgs_array_boxes = aidemo.ocr_rec_preprocess(
            input_np,
            [self.rgb888p_size[1], self.rgb888p_size[0]],
            det_boxes)
        imgs_array = imgs_array_boxes[0]
        boxes = imgs_array_boxes[1]

        # 对每个检测到的车牌进行识别 Recognize each detected license plate
        rec_res = []
        for img_array in imgs_array:
            # 配置预处理参数 Configure preprocessing parameters
            self.licence_rec.config_preprocess(
                input_image_size=[img_array.shape[3], img_array.shape[2]])
            # 执行识别 Perform recognition
            licence_str = self.licence_rec.run(img_array)
            rec_res.append(licence_str)
            gc.collect()  # 垃圾回收 Garbage collection

        return det_boxes, rec_res

    def draw_result(self, pl, det_res, rec_res):
        """
        在图像上绘制检测和识别结果
        Draw detection and recognition results on image

        参数 Parameters:
        pl: PipeLine对象 PipeLine object
        det_res: 检测结果 Detection results
        rec_res: 识别结果 Recognition results
        """
        # 清除上一帧的绘制内容 Clear previous frame drawings
        pl.osd_img.clear()

        if det_res:
            # 创建坐标数组 Create coordinates array
            point_8 = np.zeros((8), dtype=np.int16)

            # 遍历每个检测到的车牌 Iterate through each detected plate
            for det_index in range(len(det_res)):
                # 坐标转换 Coordinate conversion
                for i in range(4):
                    x = det_res[det_index][i * 2 + 0] / self.rgb888p_size[0] * self.display_size[0]
                    y = det_res[det_index][i * 2 + 1] / self.rgb888p_size[1] * self.display_size[1]
                    point_8[i * 2 + 0] = int(x)
                    point_8[i * 2 + 1] = int(y)

                # 绘制检测框 Draw detection box
                for i in range(4):
                    pl.osd_img.draw_line(
                        point_8[i * 2 + 0],
                        point_8[i * 2 + 1],
                        point_8[(i+1) % 4 * 2 + 0],
                        point_8[(i+1) % 4 * 2 + 1],
                        color=(255, 0, 255, 0),
                        thickness=4
                    )

                # 绘制识别结果文本 Draw recognition result text
                pl.osd_img.draw_string_advanced(
                    point_8[6],
                    point_8[7] + 20,
                    40,
                    rec_res[det_index],
                    color=(255,255,153,18)
                )

                pto_data = pto.get_licence_rec_data(rec_res[det_index])
                uart.send(pto_data)
                print(pto_data)
def exce_demo(pl):
    global lr

    display_mode = pl.display_mode
    rgb888p_size = pl.rgb888p_size
    display_size = pl.display_size

    # 车牌检测模型路径
    licence_det_kmodel_path="/sdcard/kmodel/LPD_640.kmodel"
    # 车牌识别模型路径
    licence_rec_kmodel_path="/sdcard/kmodel/licence_reco.kmodel"
    # 其它参数
    licence_det_input_size=[640,640]
    licence_rec_input_size=[220,32]
    confidence_threshold=0.2
    nms_threshold=0.2

    try:
        lr=LicenceRec(licence_det_kmodel_path,licence_rec_kmodel_path,det_input_size=licence_det_input_size,rec_input_size=licence_rec_input_size,confidence_threshold=confidence_threshold,nms_threshold=nms_threshold,rgb888p_size=rgb888p_size,display_size=display_size)
        while True:
            with ScopedTiming("total", 0):
                img=pl.get_frame()                  # 获取当前帧
                det_res,rec_res=lr.run(img)         # 推理当前帧
                lr.draw_result(pl,det_res,rec_res)  # 绘制当前帧推理结果
                pl.show_image()                     # 展示推理结果
                gc.collect()
    except Exception as e:
        print("车牌识别功能退出")
    finally:
        exit_demo()

def exit_demo():
    global lr
    lr.licence_det.deinit()
    lr.licence_rec.deinit()


if __name__=="__main__":

    # 显示模式，默认"hdmi",可以选择"hdmi"和"lcd"
    rgb888p_size=[640,360]
    display_size=[640,480]
    display_mode="lcd"

    # 初始化PipeLine，只关注传给AI的图像分辨率，显示的分辨率
    pl=PipeLine(rgb888p_size=rgb888p_size,display_size=display_size,display_mode=display_mode)
    pl.create()
    exce_demo(pl)

