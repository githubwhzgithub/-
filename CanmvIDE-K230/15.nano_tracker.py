# 导入所需的库和模块 Import required libraries and modules
from libs.PipeLine import PipeLine, ScopedTiming # 导入Pipeline和计时工具类 Import pipeline and timing tools
from libs.AIBase import AIBase # 导入AI基类 Import AI base class
from libs.AI2D import Ai2d # 导入AI 2D处理类 Import AI 2D processing class
from random import randint # 导入随机数生成工具 Import random number generator
import os # 导入操作系统接口模块 Import OS interface module
import ujson # 导入JSON处理模块 Import JSON processing module
from media.media import * # 导入媒体处理模块 Import media processing module
from time import * # 导入时间处理模块 Import time processing module
import nncase_runtime as nn # 导入神经网络运行时 Import neural network runtime
import ulab.numpy as np # 导入numpy兼容模块 Import numpy compatible module
import time # 导入时间模块 Import time module
import image # 导入图像处理模块 Import image processing module
import aidemo # 导入AI演示模块 Import AI demo module
import random # 导入随机数模块 Import random module
import gc # 导入垃圾回收模块 Import garbage collection module
import sys # 导入系统模块 Import system module


from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()


class TrackCropApp(AIBase):
    """
    跟踪模板任务类 Template tracking task class
    用于处理目标跟踪的模板裁剪 For processing target tracking template cropping
    """
    def __init__(self,kmodel_path,model_input_size,ratio_src_crop,center_xy_wh,rgb888p_size=[1280,720],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        # 保存kmodel路径 Save kmodel path
        self.kmodel_path=kmodel_path
        # 保存跟踪模板输入分辨率 Save tracking template input resolution
        self.model_input_size=model_input_size
        # 计算sensor给到AI的图像分辨率，宽16字节对齐
        # Calculate image resolution from sensor to AI, width aligned to 16 bytes
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        # 计算视频输出VO分辨率，宽16字节对齐
        # Calculate video output VO resolution, width aligned to 16 bytes
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        # 保存debug模式 Save debug mode
        self.debug_mode=debug_mode
        # 设置跟踪框宽、高调整系数 Set tracking box width and height adjustment coefficient
        self.CONTEXT_AMOUNT = 0.5
        # 保存src模型和crop模型输入比值 Save ratio between src model and crop model input
        self.ratio_src_crop = ratio_src_crop
        self.center_xy_wh=center_xy_wh
        # 初始化padding和crop参数列表 Initialize padding and crop parameter list
        self.pad_crop_params=[]
        # 配置两个AI2D对象用于不同的预处理操作 Configure two AI2D objects for different preprocessing operations
        self.ai2d_pad=Ai2d(debug_mode)
        self.ai2d_pad.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)
        self.ai2d_crop=Ai2d(debug_mode)
        self.ai2d_crop.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)
        self.need_pad=False

    def config_preprocess(self,input_image_size=None):
        """
        配置预处理操作
        Configure preprocessing operations
        """
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            # 初始化ai2d预处理配置 Initialize ai2d preprocessing configuration
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size

            # 计算padding和crop参数 Calculate padding and crop parameters
            self.pad_crop_params = self.get_padding_crop_param()

            # 如果需要padding,配置padding部分，否则只走crop
            # If padding is needed, configure padding part, otherwise only do crop
            if (self.pad_crop_params[0] != 0 or self.pad_crop_params[1] != 0 or
                self.pad_crop_params[2] != 0 or self.pad_crop_params[3] != 0):
                self.need_pad=True
                # 配置padding的AI2D对象 Configure AI2D object for padding
                self.ai2d_pad.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d_pad.pad([0, 0, 0, 0, self.pad_crop_params[0], self.pad_crop_params[1],
                                 self.pad_crop_params[2], self.pad_crop_params[3]], 0, [114, 114, 114])
                output_size=[self.rgb888p_size[0]+self.pad_crop_params[2]+self.pad_crop_params[3],
                           self.rgb888p_size[1]+self.pad_crop_params[0]+self.pad_crop_params[1]]
                self.ai2d_pad.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                                  [1,3,output_size[1],output_size[0]])

                # 配置crop的AI2D对象 Configure AI2D object for cropping
                self.ai2d_crop.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d_crop.crop(int(self.pad_crop_params[4]),int(self.pad_crop_params[6]),
                                  int(self.pad_crop_params[5]-self.pad_crop_params[4]+1),
                                  int(self.pad_crop_params[7]-self.pad_crop_params[6]+1))
                self.ai2d_crop.build([1,3,output_size[1],output_size[0]],
                                   [1,3,self.model_input_size[1],self.model_input_size[0]])
            else:
                self.need_pad=False
                # 只配置crop的AI2D对象 Only configure AI2D object for cropping
                self.ai2d_crop.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d_crop.crop(int(self.center_xy_wh[0]-self.pad_crop_params[8]/2.0),
                                  int(self.center_xy_wh[1]-self.pad_crop_params[8]/2.0),
                                  int(self.pad_crop_params[8]),int(self.pad_crop_params[8]))
                self.ai2d_crop.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],
                                   [1,3,self.model_input_size[1],self.model_input_size[0]])

    def preprocess(self,input_np):
        """
        执行预处理操作
        Perform preprocessing operations
        """
        if self.need_pad:
            # 如果需要padding，先执行padding再执行crop
            # If padding is needed, perform padding first then crop
            pad_output=self.ai2d_pad.run(input_np).to_numpy()
            return [self.ai2d_crop.run(pad_output)]
        else:
            # 否则直接执行crop If not, directly perform crop
            return [self.ai2d_crop.run(input_np)]

    def postprocess(self,results):
        """
        执行后处理操作
        Perform post-processing operations
        """
        with ScopedTiming("postprocess",self.debug_mode > 0):
            return results[0]

    def get_padding_crop_param(self):
        """
        计算padding和crop参数
        Calculate padding and crop parameters
        """
        # 计算模板目标框的大小 Calculate template target box size
        s_z = round(np.sqrt((self.center_xy_wh[2] + self.CONTEXT_AMOUNT * (self.center_xy_wh[2] + self.center_xy_wh[3])) *
                           (self.center_xy_wh[3] + self.CONTEXT_AMOUNT * (self.center_xy_wh[2] + self.center_xy_wh[3]))))
        # 计算中心点位置 Calculate center point position
        c = (s_z + 1) / 2

        # 计算上下左右边界 Calculate boundaries
        context_xmin = np.floor(self.center_xy_wh[0] - c + 0.5)
        context_xmax = int(context_xmin + s_z - 1)
        context_ymin = np.floor(self.center_xy_wh[1] - c + 0.5)
        context_ymax = int(context_ymin + s_z - 1)

        # 计算需要的padding大小 Calculate required padding size
        left_pad = int(max(0, -context_xmin))
        top_pad = int(max(0, -context_ymin))
        right_pad = int(max(0, int(context_xmax - self.rgb888p_size[0] + 1)))
        bottom_pad = int(max(0, int(context_ymax - self.rgb888p_size[1] + 1)))

        # 更新边界值 Update boundary values
        context_xmin = context_xmin + left_pad
        context_xmax = context_xmax + left_pad
        context_ymin = context_ymin + top_pad
        context_ymax = context_ymax + top_pad

        return [top_pad,bottom_pad,left_pad,right_pad,context_xmin,context_xmax,context_ymin,context_ymax,s_z]

    def deinit(self):
        """
        释放资源
        Release resources
        """
        with ScopedTiming("deinit",self.debug_mode > 0):
            del self.ai2d_pad
            del self.ai2d_crop
            super().deinit()

# Custom Real-time Tracking Task Class
# 自定义跟踪实时任务类
class TrackSrcApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, ratio_src_crop, rgb888p_size=[1280, 720], display_size=[1920, 1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        # kmodel path
        # kmodel路径
        self.kmodel_path = kmodel_path
        # Detection model input resolution
        # 检测模型输入分辨率
        self.model_input_size = model_input_size
        # Sensor provides AI with image resolution, 16-byte aligned width
        # sensor给到AI的图像分辨率，宽16字节对齐
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        # VO display resolution, 16-byte aligned width
        # 视频输出VO分辨率，宽16字节对齐
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        # Padding and crop parameter list
        # padding和crop参数列表
        self.pad_crop_params = []
        # Tracking box width and height adjustment coefficient
        # 跟踪框宽、高调整系数
        self.CONTEXT_AMOUNT = 0.5
        # Ratio of src model and crop model input
        # src和crop模型的输入尺寸比例
        self.ratio_src_crop = ratio_src_crop
        # debug mode
        # debug模式
        self.debug_mode = debug_mode
        # Note: The execution order of multiple pre-processing in ai2d is: crop->shift->resize/affine->pad. If it does not follow this order, you need to configure multiple ai2d objects.
        # The pre-processing of the model below needs to be resized + padded first, and then resized + cropped, so two Ai2d objects need to be configured.
        # 注意：ai2d设置多个预处理时执行的顺序为：crop->shift->resize/affine->pad，如果不符合该顺序，需要配置多个ai2d对象;
        # 如下模型预处理要先做resize+padding再做resize+crop，因此要配置两个Ai2d对象
        self.ai2d_pad = Ai2d(debug_mode)
        self.ai2d_pad.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
        self.ai2d_crop = Ai2d(debug_mode)
        self.ai2d_crop.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
        self.need_pad = False

    # Configure pre-processing operations, here we use crop, pad and resize, Ai2d supports crop/shift/pad/resize/affine, please check the code in /sdcard/app/libs/AI2D.py for details.
    # 配置预处理操作，这里使用了crop、pad和resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/app/libs/AI2D.py查看
    def config_preprocess(self, center_xy_wh, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            # Initialize the ai2d pre-processing configuration, the default is the size provided by the sensor to AI, you can modify the input size by setting input_image_size.
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，可以通过设置input_image_size自行修改输入尺寸
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            # Calculate the padding parameters and apply the pad operation to ensure that the input image size matches the model input size.
            # 计算padding参数并应用pad操作，以确保输入图像尺寸与模型输入尺寸匹配
            self.pad_crop_params = self.get_padding_crop_param(center_xy_wh)
            # If padding is required, configure the padding part, otherwise only crop.
            # 如果需要padding,配置padding部分，否则只走crop
            if (self.pad_crop_params[0] != 0 or self.pad_crop_params[1] != 0 or self.pad_crop_params[2] != 0 or self.pad_crop_params[3] != 0):
                self.need_pad = True
                self.ai2d_pad.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d_pad.pad([0, 0, 0, 0, self.pad_crop_params[0], self.pad_crop_params[1], self.pad_crop_params[2], self.pad_crop_params[3]], 0, [114, 114, 114])
                output_size = [self.rgb888p_size[0] + self.pad_crop_params[2] + self.pad_crop_params[3], self.rgb888p_size[1] + self.pad_crop_params[0] + self.pad_crop_params[1]]

                self.ai2d_pad.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]], [1, 3, output_size[1], output_size[0]])
                self.ai2d_crop.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d_crop.crop(int(self.pad_crop_params[4]), int(self.pad_crop_params[6]), int(self.pad_crop_params[5] - self.pad_crop_params[4] + 1), int(self.pad_crop_params[7] - self.pad_crop_params[6] + 1))
                self.ai2d_crop.build([1, 3, output_size[1], output_size[0]], [1, 3, self.model_input_size[1], self.model_input_size[0]])
            else:
                self.need_pad = False
                self.ai2d_crop.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
                self.ai2d_crop.crop(int(center_xy_wh[0] - self.pad_crop_params[8] / 2.0), int(center_xy_wh[1] - self.pad_crop_params[8] / 2.0), int(self.pad_crop_params[8]), int(self.pad_crop_params[8]))
                self.ai2d_crop.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]], [1, 3, self.model_input_size[1], self.model_input_size[0]])

    # Rewrite the preprocess function, because this part is not a simple one-pass ai2d preprocessing, so this function needs to be rewritten.
    # 重写预处理函数preprocess，因为该部分不是单纯的走一个ai2d做预处理，所以该函数需要重写
    def preprocess(self, input_np):
        with ScopedTiming("preprocess", self.debug_mode > 0):
            if self.need_pad:
                pad_output = self.ai2d_pad.run(input_np).to_numpy()
                return [self.ai2d_crop.run(pad_output)]
            else:
                return [self.ai2d_crop.run(input_np)]

    # Custom post-processing, results is a list of model output arrays.
    # 自定义后处理，results是模型输出array的列表
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            return results[0]

    # Calculate padding and crop parameters.
    # 计算padding和crop参数
    def get_padding_crop_param(self, center_xy_wh):
        s_z = round(np.sqrt((center_xy_wh[2] + self.CONTEXT_AMOUNT * (center_xy_wh[2] + center_xy_wh[3])) * (center_xy_wh[3] + self.CONTEXT_AMOUNT * (center_xy_wh[2] + center_xy_wh[3]))) * self.ratio_src_crop)
        c = (s_z + 1) / 2
        context_xmin = np.floor(center_xy_wh[0] - c + 0.5)
        context_xmax = int(context_xmin + s_z - 1)
        context_ymin = np.floor(center_xy_wh[1] - c + 0.5)
        context_ymax = int(context_ymin + s_z - 1)
        left_pad = int(max(0, -context_xmin))
        top_pad = int(max(0, -context_ymin))
        right_pad = int(max(0, int(context_xmax - self.rgb888p_size[0] + 1)))
        bottom_pad = int(max(0, int(context_ymax - self.rgb888p_size[1] + 1)))
        context_xmin = context_xmin + left_pad
        context_xmax = context_xmax + left_pad
        context_ymin = context_ymin + top_pad
        context_ymax = context_ymax + top_pad
        return [top_pad, bottom_pad, left_pad, right_pad, context_xmin, context_xmax, context_ymin, context_ymax, s_z]

    # Rewrite deinit
    # 重写deinit
    def deinit(self):
        with ScopedTiming("deinit", self.debug_mode > 0):
            del self.ai2d_pad
            del self.ai2d_crop
            super().deinit()

# Tracker App Class
# 跟踪模型应用类
class TrackerApp(AIBase):
    def __init__(self, kmodel_path, crop_input_size, thresh, rgb888p_size=[1280, 720], display_size=[1920, 1080], debug_mode=0):
        super().__init__(kmodel_path, rgb888p_size, debug_mode)
        # kmodel path
        # kmodel路径
        self.kmodel_path = kmodel_path
        # Crop model input size
        # crop模型的输入尺寸
        self.crop_input_size = crop_input_size
        # Tracking box threshold
        # 跟踪框阈值
        self.thresh = thresh
        # Tracking box width and height adjustment coefficient
        # 跟踪框宽、高调整系数
        self.CONTEXT_AMOUNT = 0.5
        # Sensor provides AI with image resolution, 16-byte aligned width
        # sensor给到AI的图像分辨率，宽16字节对齐
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        # VO display resolution, 16-byte aligned width
        # 视频输出VO分辨率，宽16字节对齐
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        # debug mode
        # debug模式
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            pass

    # Rewrite the run function, because there is no pre-processing process, so the original run operation containing preprocess->inference->postprocess is not suitable, here only contains inference->postprocess.
    # 重写run函数，因为没有预处理过程，所以原来run操作中包含的preprocess->inference->postprocess不合适，这里只包含inference->postprocess
    def run(self, input_np_1, input_np_2, center_xy_wh):
        input_tensors = []
        input_tensors.append(nn.from_numpy(input_np_1))
        input_tensors.append(nn.from_numpy(input_np_2))
        results = self.inference(input_tensors)
        return self.postprocess(results, center_xy_wh)

    # Custom post-processing, results is a list of model output arrays, here we use the nanotracker_postprocess list from aidemo.
    # 自定义后处理，results是模型输出array的列表,这里使用了aidemo的nanotracker_postprocess列表
    def postprocess(self, results, center_xy_wh):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            det = aidemo.nanotracker_postprocess(results[0], results[1], [self.rgb888p_size[1], self.rgb888p_size[0]], self.thresh, center_xy_wh, self.crop_input_size[0], self.CONTEXT_AMOUNT)
            return det


class NanoTracker:
    """
    NanoTracker类: 实现目标追踪功能
    NanoTracker class: Implements object tracking functionality
    """
    def __init__(self,track_crop_kmodel,track_src_kmodel,tracker_kmodel,crop_input_size,src_input_size,threshold=0.25,rgb888p_size=[1280,720],display_size=[1920,1080],debug_mode=0):
        """
        初始化函数 / Initialization function

        参数 / Parameters:
        track_crop_kmodel: 模板裁剪模型路径 / Path to template cropping model
        track_src_kmodel: 源图像追踪模型路径 / Path to source tracking model
        tracker_kmodel: 追踪器模型路径 / Path to tracker model
        crop_input_size: 裁剪模型输入尺寸 / Input size for crop model
        src_input_size: 源模型输入尺寸 / Input size for source model
        threshold: 追踪阈值 / Tracking threshold
        rgb888p_size: RGB图像尺寸 / RGB image size
        display_size: 显示尺寸 / Display size
        debug_mode: 调试模式 / Debug mode
        """
        self.track_crop_kmodel = track_crop_kmodel  # 跟踪模版模型路径 / Path to tracking template model
        self.track_src_kmodel = track_src_kmodel    # 跟踪实时模型路径 / Path to real-time tracking model
        self.tracker_kmodel = tracker_kmodel        # 跟踪模型路径 / Path to tracker model
        self.crop_input_size = crop_input_size      # 跟踪模版模型输入分辨率 / Input resolution for template model
        self.src_input_size = src_input_size        # 跟踪实时模型输入分辨率 / Input resolution for real-time model
        self.threshold = threshold                   # 阈值 / Threshold

        # 追踪相关参数 / Tracking related parameters
        self.CONTEXT_AMOUNT = 0.5       # 跟踪框宽、高调整系数 / Tracking box width and height adjustment coefficient
        self.ratio_src_crop = 0.0      # src模型和crop模型输入比值 / Ratio between src and crop model inputs
        self.track_x1 = float(600)     # 起始跟踪目标框左上角点x / Initial tracking box top-left x coordinate
        self.track_y1 = float(300)     # 起始跟踪目标框左上角点y / Initial tracking box top-left y coordinate
        self.track_w = float(100)      # 起始跟踪目标框宽度 / Initial tracking box width
        self.track_h = float(100)      # 起始跟踪目标框高度 / Initial tracking box height

        # 追踪状态存储列表 / Tracking state storage lists
        self.draw_mean = []            # 初始目标框位置列表 / Initial target box position list
        self.center_xy_wh = []         # 中心点坐标和宽高 / Center coordinates, width and height
        self.track_boxes = []          # 追踪框列表 / Tracking box list
        self.center_xy_wh_tmp = []     # 临时中心点坐标和宽高 / Temporary center coordinates, width and height
        self.track_boxes_tmp = []      # 临时追踪框列表 / Temporary tracking box list

        # 模型输出 / Model outputs
        self.crop_output = None        # 裁剪模型输出 / Crop model output
        self.src_output = None         # 源模型输出 / Source model output

        # 初始化时间控制 / Initialization time control
        self.seconds = 8               # 初始化持续时间(秒) / Initialization duration (seconds)
        self.endtime = time.ticks_ms()//1000 + self.seconds
        self.enter_init = True         # 初始化标志 / Initialization flag

        # 图像处理参数 / Image processing parameters
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]  # sensor给到AI的图像分辨率，宽16字节对齐 / Image resolution from sensor to AI, width aligned to 16 bytes
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]   # 视频输出VO分辨率，宽16字节对齐 / Video output resolution, width aligned to 16 bytes

        self.init_param()  # 初始化参数 / Initialize parameters

        # 初始化追踪器组件 / Initialize tracker components
        self.track_crop = TrackCropApp(
            self.track_crop_kmodel,
            model_input_size=self.crop_input_size,
            ratio_src_crop=self.ratio_src_crop,
            center_xy_wh=self.center_xy_wh,
            rgb888p_size=self.rgb888p_size,
            display_size=self.display_size,
            debug_mode=0
        )
        self.track_src = TrackSrcApp(
            self.track_src_kmodel,
            model_input_size=self.src_input_size,
            ratio_src_crop=self.ratio_src_crop,
            rgb888p_size=self.rgb888p_size,
            display_size=self.display_size,
            debug_mode=0
        )
        self.tracker = TrackerApp(
            self.tracker_kmodel,
            crop_input_size=self.crop_input_size,
            thresh=self.threshold,
            rgb888p_size=self.rgb888p_size,
            display_size=self.display_size
        )
        self.track_crop.config_preprocess()

    def run(self, input_np):
        """
        运行追踪算法 / Run tracking algorithm

        Args:
            input_np: 输入图像 / Input image

        Returns:
            追踪结果 / Tracking results
        """
        nowtime = time.ticks_ms()//1000
        # 初始化阶段：获取模板特征 / Initialization phase: get template features
        if (self.enter_init and nowtime <= self.endtime):
            print("倒计时 / Countdown: " + str(self.endtime - nowtime) + " 秒/seconds")
            self.crop_output = self.track_crop.run(input_np)
            time.sleep(1)
            return self.draw_mean
        # 追踪阶段：对当前帧进行特征提取和追踪 / Tracking phase: feature extraction and tracking for current frame
        else:
            self.track_src.config_preprocess(self.center_xy_wh)
            self.src_output = self.track_src.run(input_np)
            det = self.tracker.run(self.crop_output, self.src_output, self.center_xy_wh)
            return det

    def draw_result(self, pl, box):
        """
        绘制追踪结果 / Draw tracking results

        Args:
            pl: Pipeline对象 / Pipeline object
            box: 追踪框坐标 / Tracking box coordinates
        """
        pl.osd_img.clear()
        # 初始化阶段绘制 / Drawing during initialization phase
        if self.enter_init:
            pl.osd_img.draw_rectangle(box[0], box[1], box[2], box[3], color=(255, 0, 255, 0), thickness=4)
            if (time.ticks_ms()//1000 > self.endtime):
                self.enter_init = False
        # 追踪阶段绘制 / Drawing during tracking phase
        else:
            self.track_boxes = box[0]
            self.center_xy_wh = box[1]

            # 检查追踪框是否有效 / Check if tracking box is valid
            track_bool = True
            if (len(self.track_boxes) != 0):
                # 确保追踪框在图像范围内 / Ensure tracking box is within image bounds
                track_bool = (self.track_boxes[0] > 10 and
                            self.track_boxes[1] > 10 and
                            self.track_boxes[0] + self.track_boxes[2] < self.rgb888p_size[0] - 10 and
                            self.track_boxes[1] + self.track_boxes[3] < self.rgb888p_size[1] - 10)
            else:
                track_bool = False

            # 检查目标大小是否合适 / Check if target size is appropriate
            if (len(self.center_xy_wh) != 0):
                track_bool = track_bool and self.center_xy_wh[2] * self.center_xy_wh[3] < 40000
            else:
                track_bool = False

            if (track_bool):
                # 更新临时存储并绘制有效的追踪框 / Update temporary storage and draw valid tracking box
                self.center_xy_wh_tmp = self.center_xy_wh
                self.track_boxes_tmp = self.track_boxes
                x1 = int(float(self.track_boxes[0]) * self.display_size[0] / self.rgb888p_size[0])
                y1 = int(float(self.track_boxes[1]) * self.display_size[1] / self.rgb888p_size[1])
                w = int(float(self.track_boxes[2]) * self.display_size[0] / self.rgb888p_size[0])
                h = int(float(self.track_boxes[3]) * self.display_size[1] / self.rgb888p_size[1])
                pl.osd_img.draw_rectangle(x1, y1, w, h, color=(255, 255, 0, 0), thickness=4)
                pto_data = pto.get_nano_tracker_data(x1, y1, w, h)
                uart.send(pto_data)
                print(pto_data)
            else:
                # 使用上一帧的有效追踪框并显示警告 / Use previous valid tracking box and show warnings
                self.center_xy_wh = self.center_xy_wh_tmp
                self.track_boxes = self.track_boxes_tmp
                x1 = int(float(self.track_boxes[0]) * self.display_size[0] / self.rgb888p_size[0])
                y1 = int(float(self.track_boxes[1]) * self.display_size[1] / self.rgb888p_size[1])
                w = int(float(self.track_boxes[2]) * self.display_size[0] / self.rgb888p_size[0])
                h = int(float(self.track_boxes[3]) * self.display_size[1] / self.rgb888p_size[1])
                pl.osd_img.draw_rectangle(x1, y1, w, h, color=(255, 255, 0, 0), thickness=4)
                pl.osd_img.draw_string_advanced(x1, y1-50, 32, "请远离摄像头，保持跟踪物体大小基本一致! / Please move away from camera, keep target size consistent!", color=(255, 255, 0, 0))
                pl.osd_img.draw_string_advanced(x1, y1-100, 32, "请靠近中心! / Please move closer to center!", color=(255, 255, 0, 0))

    def init_param(self):
        """
        初始化追踪参数 / Initialize tracking parameters
        """
        # 计算src和crop模型的输入尺寸比例 / Calculate ratio between src and crop model input sizes
        self.ratio_src_crop = float(self.src_input_size[0])/float(self.crop_input_size[0])
        print(self.ratio_src_crop)

        # 检查初始追踪框是否在有效范围内 / Check if initial tracking box is within valid range
        if (self.track_x1 < 50 or self.track_y1 < 50 or
            self.track_x1+self.track_w >= self.rgb888p_size[0]-50 or
            self.track_y1+self.track_h >= self.rgb888p_size[1]-50):
            print("**剪切范围超出图像范围** / **Crop range exceeds image boundaries**")
        else:
            # 计算追踪框中心点 / Calculate tracking box center point
            track_mean_x = self.track_x1 + self.track_w / 2.0
            track_mean_y = self.track_y1 + self.track_h / 2.0

            # 计算显示尺寸下的追踪框参数 / Calculate tracking box parameters in display size
            draw_mean_w = int(self.track_w / self.rgb888p_size[0] * self.display_size[0])
            draw_mean_h = int(self.track_h / self.rgb888p_size[1] * self.display_size[1])
            draw_mean_x = int(track_mean_x / self.rgb888p_size[0] * self.display_size[0] - draw_mean_w / 2.0)
            draw_mean_y = int(track_mean_y / self.rgb888p_size[1] * self.display_size[1] - draw_mean_h / 2.0)

            # 初始化各种追踪参数 / Initialize various tracking parameters
            self.draw_mean = [draw_mean_x, draw_mean_y, draw_mean_w, draw_mean_h]
            self.center_xy_wh = [track_mean_x, track_mean_y, self.track_w, self.track_h]
            self.center_xy_wh_tmp = [track_mean_x, track_mean_y, self.track_w, self.track_h]
            self.track_boxes = [self.track_x1, self.track_y1, self.track_w, self.track_h, 1]
            self.track_boxes_tmp = np.array([self.track_x1, self.track_y1, self.track_w, self.track_h, 1])


def exce_demo(pl):
    """
    执行追踪演示 / Execute tracking demonstration

    Args:
        pl: Pipeline对象 / Pipeline object
    """
    global track

    # 获取显示相关参数 / Get display-related parameters
    display_mode = pl.display_mode
    rgb888p_size = pl.rgb888p_size
    display_size = pl.display_size

    # 模型文件路径配置 / Model file path configuration
    track_crop_kmodel_path = "/sdcard/kmodel/cropped_test127.kmodel"    # 跟踪模板模型路径 / Path to tracking template model
    track_src_kmodel_path = "/sdcard/kmodel/nanotrack_backbone_sim.kmodel"    # 跟踪实时模型路径 / Path to real-time tracking model
    tracker_kmodel_path = "/sdcard/kmodel/nanotracker_head_calib_k230.kmodel"    # 跟踪模型路径 / Path to tracker model

    # 追踪参数配置 / Tracking parameter configuration
    track_crop_input_size = [127, 127]    # 模板模型输入尺寸 / Template model input size
    track_src_input_size = [255, 255]     # 源模型输入尺寸 / Source model input size
    threshold = 0.01                        # 追踪阈值 / Tracking threshold

    try:
        # 初始化追踪器 / Initialize tracker
        track = NanoTracker(track_crop_kmodel_path, track_src_kmodel_path, tracker_kmodel_path,
                          crop_input_size=track_crop_input_size,
                          src_input_size=track_src_input_size,
                          threshold=threshold,
                          rgb888p_size=rgb888p_size,
                          display_size=display_size)

        while True:
            with ScopedTiming("total", 0):
                img = pl.get_frame()              # 获取当前帧 / Get current frame
                output = track.run(img)           # 推理当前帧 / Process current frame
                track.draw_result(pl, output)     # 绘制当前帧推理结果 / Draw inference results
                pl.show_image()                   # 展示推理结果 / Display results
                gc.collect()                      # 垃圾回收 / Garbage collection

    except Exception as e:
        print("物体追踪功能退出 / Object tracking function exited", e)
    finally:
        exit_demo()                              # 退出清理 / Cleanup on exit

def exit_demo():
    """
    退出演示并清理资源 / Exit demonstration and cleanup resources
    """
    global track
    track.track_crop.deinit()     # 释放裁剪模型资源 / Release crop model resources
    track.track_src.deinit()      # 释放源模型资源 / Release source model resources
    track.tracker.deinit()        # 释放追踪器资源 / Release tracker resources


if __name__ == "__main__":
    # 配置基本参数 / Configure basic parameters
    rgb888p_size = [1280, 720]    # RGB图像尺寸 / RGB image size
    display_size = [640, 480]     # 显示尺寸 / Display size
    display_mode = "lcd"          # 显示模式 / Display mode

    # 初始化并运行Pipeline / Initialize and run Pipeline
    pl = PipeLine(rgb888p_size=rgb888p_size,
                 display_size=display_size,
                 display_mode=display_mode)
    pl.create()
    exce_demo(pl)

