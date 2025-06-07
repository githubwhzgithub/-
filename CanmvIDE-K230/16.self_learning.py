# -*- coding: utf-8 -*-
"""
本代码实现了一个自学习应用，用于对摄像头采集的图像进行模型推理、特征采集及后续特征匹配
This code implements a self-learning application for performing model inference on frames captured by a sensor,
feature collection, and subsequent feature matching.
"""

# 导入依赖的库和模块
# Import necessary libraries and modules
from libs.PipeLine import PipeLine, ScopedTiming  # Pipeline和计时工具 // pipeline and timing tool
from libs.AIBase import AIBase  # 基础的AI类 // the base AI class
from libs.AI2D import Ai2d  # 用于图像预处理的Ai2d模块 // Ai2d module for image preprocessing
import os  # 操作系统接口 // operating system interfaces
import ujson  # 快速JSON解析 // fast JSON parsing
from media.media import *  # 媒体处理模块 // media processing module
from time import *  # 时间模块 // time module
import nncase_runtime as nn  # nncase运行时 // nncase runtime module
import ulab.numpy as np  # ulab下的numpy，用于嵌入式数值计算 // ulab numpy for embedded numerical computation
import time  # 标准时间模块 // standard time module
import utime  # 微型嵌入式时间模块 // micro-time module for embedded systems
import image  # 图像处理模块 // image processing module
import random  # 随机数模块 // random number module
import gc  # 垃圾回收模块 // garbage collection module
import sys  # 系统模块 // system-specific parameters and functions
import aicube  # aicube模块, 可用于AI算子 // aicube module, can be used for AI operators

from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
# uart = None
uart = YbUart(baudrate=115200)
pto = YbProtocol()


# 自定义自学习类
# Custom self-learning class
class SelfLearningApp(AIBase):
    """
    自定义自学习应用类，继承自AIBase基类
    Custom self-learning application class extending AIBase.
    """
    def __init__(self, kmodel_path, model_input_size, labels, top_k, threshold, database_path,
                 rgb888p_size=[224,224], display_size=[1920,1080], debug_mode=0, recong_only=False):
        """
        初始化自学习应用
        Initialize the self-learning application.

        参数说明 / Parameters:
          kmodel_path (str): 模型文件路径 / Path to the model file.
          model_input_size (list): 模型输入尺寸 [width, height] / Model input dimensions.
          labels (list): 类别标签列表 / List of label names.
          top_k (int): 选择相似度最高的前K个结果 / Select top K results with highest similarity.
          threshold (float): 特征匹配阈值 / Threshold for feature matching.
          database_path (str): 保存特征文件的目录路径 / Directory path to store feature files.
          rgb888p_size (list): sensor提供图像尺寸，默认[224,224] / Sensor provided image resolution, default [224,224].
          display_size (list): 显示图像的分辨率 / Display image resolution.
          debug_mode (int): 调试模式开关 / Debug mode switch.
          recong_only (bool): 仅识别模式开关 / Recognition-only mode switch.
        """
        # 调用父类的构造函数
        # Call the parent class constructor
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path  # 模型文件路径 // path of the model file

        # 模型输入分辨率
        # Model input resolution
        self.model_input_size = model_input_size

        self.labels = labels  # 类别标签列表 // list of category labels
        self.database_path = database_path  # 存储特征的数据库路径 // path to store features

        # sensor给到AI的图像分辨率，调整宽度为16的倍数
        # Sensor provides image resolution; align width to multiples of 16
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]

        # 显示图像的分辨率，调整宽度为16的倍数
        # Display resolution; align width to multiples of 16
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]

        self.debug_mode = debug_mode  # 调试模式标志 // debug mode flag

        # 识别阈值，用于特征匹配
        # Recogniton threshold for feature matching
        self.threshold = threshold

        # 选择top_k个相似度大于阈值的结果类别
        # Choose top_k results with similarity above the threshold
        self.top_k = top_k

        # 对应类别注册特征数量，每个类别需要采集的特征文件对数
        # Number of features to be registered per category
        self.features = [2, 2]

        # 单个特征采集之间间隔的帧数
        # Interval (in frame count) between two feature collections
        self.time_one = 60

        # 初始化时间累积变量
        # Initialize time accumulation variables
        self.time_all = 0
        self.time_now = 0

        # 类别索引，从0开始，用于依次注册不同类别
        # Category index; used to register features for different categories sequentially
        self.category_index = 0

        # 特征化部分剪切区域的宽高设置
        # Cropping dimensions (width and height) for feature extraction
        self.crop_w = 400
        self.crop_h = 400

        # 根据图像中心计算crop的位置
        # Calculate crop position based on provided image resolution (center crop)
        self.crop_x = self.rgb888p_size[0] / 2.0 - self.crop_w / 2.0
        self.crop_y = self.rgb888p_size[1] / 2.0 - self.crop_h / 2.0

        # 定义画面上输出显示使用的crop位置信息(初始化为0，后续在data_init中计算)
        # Define crop position information for on-screen display (OSD); will be updated later in data_init
        self.crop_x_osd = 0
        self.crop_y_osd = 0
        self.crop_w_osd = 0
        self.crop_h_osd = 0

        # 创建Ai2d实例，用于实现模型输入前的预处理操作
        # Create an Ai2d instance for model preprocessing
        self.ai2d = Ai2d(debug_mode)

        # 设置Ai2d的输入输出格式和数据类型
        # Set Ai2d configuration for input/output formats and data types
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

        # 是否只识别模式，跳过特征采集等操作
        # Whether to run in recognition-only mode (skip feature collection etc.)
        self.recong_only = recong_only

        # 初始化数据（例如创建存储特征的文件夹、计算OSD窗口位置等）
        # Initialize data (e.g. create directory for storing features and calculate OSD positions)
        self.data_init()

    # 配置预处理操作
    # Configure preprocessing operations
    # 这里使用了crop和resize，Ai2d支持crop/shift/pad/resize/affine
    # Here we perform crop and resize; Ai2d supports crop/shift/pad/resize/affine operations.
    def config_preprocess(self, input_image_size=None):
        """
        配置图像预处理操作
        Configure image preprocessing operations.

        参数:
          input_image_size (list, optional): 输入图像尺寸，可自定义，默认为sensor尺寸 / Optional custom input size; defaults to sensor size
        """
        # 使用ScopedTiming来处理预处理配置的耗时统计（仅在debug_mode开启时）
        # Use ScopedTiming for timing this configuration (only in debug mode)
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            # 根据传入的input_image_size更新输入尺寸，否则使用rgb888p_size
            # Update the ai2d input size based on parameter input_image_size, otherwise use self.rgb888p_size
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            # 配置crop操作，剪切指定区域
            # Set the crop operation to extract a specified region from the input image
            self.ai2d.crop(int(self.crop_x), int(self.crop_y), int(self.crop_w), int(self.crop_h))
            # 配置resize操作，使用tensorflow双线性插值和half_pixel采样模式
            # Set the resize operation using TensorFlow bilinear interpolation with half_pixel mode
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            # 构建Ai2d预处理Pipeline，指定输入输出的张量尺寸 [batch, channels, height, width]
            # Build the Ai2d preprocessing pipeline, specifying input and output tensor shapes [N, C, H, W]
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]]
            )

    # 自定义当前任务的后处理
    # Custom postprocessing for current task
    def postprocess(self, results):
        """
        对模型推理结果进行后处理
        Perform postprocessing on the model results.

        参数:
          results (list): 模型推理返回的结果列表 / List of results returned by the inference.

        返回:
          处理后的结果 / The processed result.
        """
        with ScopedTiming("postprocess", self.debug_mode > 0):
            # 这里只返回results[0][0]，表示只取最主要的结果
            # For simplicity, just return results[0][0] as the primary output.
            return results[0][0]

    # 绘制结果，绘制特征采集框和特征分类框
    # Draw results displaying feature collection and classification boxes
    def draw_result(self, pl, feature):
        """
        绘制检测结果到屏幕
        Draw the detection results on the screen.

        参数:
          pl (PipeLine): Pipeline示例，用于显示OSD图像 // Pipeline instance for displaying OSD image.
          feature (array): 当前帧提取的特征向量 // Feature vector extracted from current frame.
        """
        # 清空OSD图像
        # Clear the on-screen display (OSD) image
        pl.osd_img.clear()
        with ScopedTiming("display_draw", self.debug_mode > 0):
            # 绘制特征采集框（黄色矩形框）
            # Draw the rectangle (in yellow) indicating the feature collection region
            pl.osd_img.draw_rectangle(
                self.crop_x_osd, self.crop_y_osd,
                self.crop_w_osd, self.crop_h_osd,
                color=(255, 255, 0, 255), thickness=4
            )
            # 判断是否处于特征采集模式
            # Check if we are in feature collection mode (not recognition only)
            if (not self.recong_only and self.category_index < len(self.labels)):
                # 更新时间计数，用于统计采集间隔
                # Increment time counter for feature collection interval
                self.time_now += 1
                # 绘制采集提示文字
                # Draw the prompt text indicating which category is being collected and collection count
                pl.osd_img.draw_string_advanced(
                    50, self.crop_y_osd - 50, 20,
                    "请将待添加类别放入框内进行特征采集：" + self.labels[self.category_index] + "_" + str(int(self.time_now-1) // self.time_one) + ".bin",
                    color=(255,255,0,0)
                )
                # 保存当前特征到指定文件
                # Save the current feature to a file for later matching
                with open(self.database_path + self.labels[self.category_index] + "_" + str(int(self.time_now-1) // self.time_one) + ".bin", 'wb') as f:
                    f.write(feature.tobytes())
                # 若达到该类别预设的特征采集数量，则切换到下一个类别
                # If the number of collected features for this category reaches the preset count, move to the next category
                if (self.time_now // self.time_one == self.features[self.category_index]):
                    self.category_index += 1
                    self.time_all -= self.time_now
                    self.time_now = 0
            else:
                # 如果不是采集特征模式，则进行特征匹配，显示匹配的类别和得分
                # Otherwise, in recognition mode, perform feature matching and show matching category and score.
                results_learn = []
                # 列出数据库中所有feature文件
                # List all feature files in the database directory
                list_features = os.listdir(self.database_path)
                for feature_name in list_features:
                    # 打开feature文件并读取数据
                    # Open the feature file and read its contents
                    with open(self.database_path + feature_name, 'rb') as f:
                        data = f.read()
                    # 将二进制数据转为特征向量
                    # Convert binary data into a feature vector using ulab numpy frombuffer
                    save_vec = np.frombuffer(data, dtype=np.float)
                    # 计算当前特征和保存特征之间的相似度
                    # Calculate similarity between the current feature and the stored feature
                    score = self.getSimilarity(feature, save_vec)
                    # 如果相似度大于预设阈值，则认为匹配上该类别
                    # If the similarity is above the threshold, consider it as a match.
                    if (score > self.threshold):
                        res = feature_name.split("_")
                        is_same = False
                        # 合并同一类别结果，保留相似度更高的结果
                        # Merge the same category results: update if current score is higher.
                        for r in results_learn:
                            if (r["category"] == res[0]):
                                if (r["score"] < score):
                                    r["bin_file"] = feature_name
                                    r["score"] = score
                                is_same = True
                        if (not is_same):
                            # 如果当前结果列表长度小于top_k，直接添加
                            # If current result list is smaller than top_k, add the new match directly.
                            if (len(results_learn) < self.top_k):
                                evec = {}
                                evec["category"] = res[0]
                                evec["score"] = score
                                evec["bin_file"] = feature_name
                                results_learn.append(evec)
                                # 按得分降序排列
                                # Sort the results in descending order by score
                                results_learn = sorted(results_learn, key=lambda x: -x["score"])
                            else:
                                # 如果已达到top_k，但当前得分更高，则进行替换
                                # If top_k results are already present but the new score is higher than the lowest score, replace.
                                if score <= results_learn[self.top_k - 1]["score"]:
                                    continue
                                else:
                                    evec = {}
                                    evec["category"] = res[0]
                                    evec["score"] = score
                                    evec["bin_file"] = feature_name
                                    results_learn.append(evec)
                                    results_learn = sorted(results_learn, key=lambda x: -x["score"])
                                    # 删除最后一个元素以保持长度为top_k
                                    # Remove the element with the lowest score to maintain top_k results.
                                    results_learn.pop()
                # 绘制匹配结果文字信息，依次显示每个匹配类别及其得分
                # Draw the matching result texts: each matching category and its similarity score.
                draw_y = 200
                for r in results_learn:
                    pl.osd_img.draw_string_advanced(
                        50, draw_y, 20,
                        r["category"] + " : " + str(r["score"]), color=(255,255,0,0)
                    )
                    draw_y += 50

                    pto_data = pto.get_self_learning_data(r["category"], r["score"])
                    uart.send(pto_data)
                    print(pto_data)


    # 数据初始化：创建文件夹、计算OSD位置以及计算总采集时间
    # Data initialization: create necessary directories, compute OSD positions and total collection time.
    def data_init(self):
        """
        数据初始化，主要包括创建存储特征的目录和计算OSD上显示的剪切区域位置
        Initialize data. Mainly creates the directory for features and calculates the on-screen crop positions.
        """
        try:
            # 尝试创建数据库文件夹（用于存储特征文件）
            # Try creating the database directory for storing feature files.
            os.mkdir(self.database_path)
        except Exception as e:
            print(e)
        # 计算OSD显示区域位置，将摄像头图像区域按显示分辨率进行缩放转换
        # Calculate the positions for the OSD display region by scaling from sensor resolution to display resolution.
        self.crop_x_osd = int(self.crop_x / self.rgb888p_size[0] * self.display_size[0])
        self.crop_y_osd = int(self.crop_y / self.rgb888p_size[1] * self.display_size[1])
        self.crop_w_osd = int(self.crop_w / self.rgb888p_size[0] * self.display_size[0])
        self.crop_h_osd = int(self.crop_h / self.rgb888p_size[1] * self.display_size[1])
        # 累计所有类别所需采集的帧数，供后续时间控制
        # Sum up all the required frames to be collected from all categories.
        for i in range(len(self.labels)):
            for j in range(self.features[i]):
                self.time_all += self.time_one

    # 计算两个特征向量的余弦相似度
    # Compute the cosine similarity between two feature vectors.
    def getSimilarity(self, output_vec, save_vec):
        """
        计算两个特征向量之间的余弦相似度
        Compute the cosine similarity between two feature vectors.

        参数:
          output_vec: 当前帧提取出的特征向量 / Feature vector from current frame.
          save_vec:  存储的特征向量 / Stored feature vector.

        返回:
          余弦相似度值 / Cosine similarity value.
        """
        # 计算内积
        # Compute inner product.
        tmp = sum(output_vec * save_vec)
        # 计算输出特征向量的模长
        # Compute norm of output vector.
        mold_out = np.sqrt(sum(output_vec * output_vec))
        # 计算保存向量的模长
        # Compute norm of saved vector.
        mold_save = np.sqrt(sum(save_vec * save_vec))
        # 返回归一化的相似度即余弦相似度
        # Return the cosine similarity.
        return tmp / (mold_out * mold_save)

# 自学习例程执行函数
# Self-learning demo execution function
def exce_demo(pl, recong_only=False):
    """
    自学习例程主要流程：
      1. 初始化自学习实例
      2. 启动图像采集、推理、特征采集及后续匹配展示
    Main execution flow for self-learning demo:
      1. Initialize self-learning instance.
      2. Start continuous frame capture, inference, feature collection, and matching display.

    参数:
      pl (PipeLine): Pipeline实例，用于图像采集和显示 // Pipeline instance for frame capture and display.
      recong_only (bool): 是否仅作为识别模式，不进行特征采集 // Whether to run in recognition-only mode.
    """
    global sl, database_path

    # 获取显示和图像参数
    # Retrieve display and image sizes
    display_mode = pl.display_mode
    rgb888p_size = pl.rgb888p_size
    display_size = pl.display_size

    # 模型路径设置
    # Set model path
    kmodel_path = "/sdcard/kmodel/recognition.kmodel"
    # 特征存储路径
    # Database (feature storage) path.
    database_path = "/sdcard/utils/features/"
    # 其它参数设置，例如模型输入尺寸、类别标签、top_k以及匹配阈值
    # Other parameters: model input size, labels, top_k and threshold.
    model_input_size = [224,224]
    # labels = ["耳机", "超声波模块"]
    labels = ["earphone", "ultrasonic"]
    top_k = 3
    threshold = 0.45

    try:
        # 初始化自学习实例
        # Initialize the SelfLearningApp instance
        sl = SelfLearningApp(
            kmodel_path,
            model_input_size=model_input_size,
            labels=labels,
            top_k=top_k,
            threshold=threshold,
            database_path=database_path,
            rgb888p_size=rgb888p_size,
            display_size=display_size,
            debug_mode=0,
            recong_only=recong_only
        )
        # 配置预处理操作
        # Configure the preprocessing operations
        sl.config_preprocess()
        # 无限循环处理每一帧数据
        # Infinite loop to process each frame
        while True:
            with ScopedTiming("total", 0):
                # 获取当前帧图像数据
                # Capture the current frame from the sensor
                img = pl.get_frame()
                # 推理当前帧图像，得到输出特征
                # Run inference on the current frame to get features
                res = sl.run(img)
                # 绘制结果到Pipeline的OSD图像中（包含特征采集框等信息）
                # Draw the results (feature collection box and matching info) onto the pipeline's OSD image.
                sl.draw_result(pl, res)
                # 显示当前的图像及绘制的OSD信息
                # Display the image with the drawn OSD information.
                pl.show_image()
                # 主动调用垃圾回收，释放内存
                # Explicitly call garbage collection to free memory.
                gc.collect()
    except Exception as e:
        # 捕获异常后打印提示信息
        # Print message when an exception is caught.
        print("自学习例程退出")
    finally:
        # 最终退出时执行反初始化
        # Finally, deinitialize the demo.
        exit_demo()

# 退出例程，反初始化资源并清除特征保存文件夹
# Exit routine: cleanup resources and remove stored feature files
def exit_demo():
    global sl, database_path
    # 删除features文件夹下所有文件，并删除文件夹本身
    # Remove all files within the features directory and the directory itself
    stat_info = os.stat(database_path)
    # 判断目录是否为文件夹（目录标志位0x4000）
    # Check if database_path is a directory based on stat info (0x4000 indicates a directory)
    if (stat_info[0] & 0x4000):
        list_files = os.listdir(database_path)
        for l in list_files:
            os.remove(database_path + l)
    os.rmdir(database_path)
    # 执行自学习实例的反初始化，释放资源
    # Deinitialize the self-learning instance and free up resources.
    sl.deinit()

# 主程序入口
# Main entry point
if __name__ == "__main__":
    # 设置sensor的图像尺寸和显示尺寸
    # Set sensor image size and display size
    rgb888p_size = [1280,960]
    display_size = [640, 480]
    display_mode = "lcd"

    # 初始化PipeLine实例用于实现传感器图像采集和OSD显示
    # Initialize the PipeLine instance for sensor image capture and OSD display.
    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode)
    pl.create()
    # 执行自学习例程, 第二个参数False表示非仅识别模式，即包括特征采集功能
    # Execute the self-learning demo. The second parameter False indicates not recognition-only mode,
    # so feature collection is performed.
    exce_demo(pl, False)
