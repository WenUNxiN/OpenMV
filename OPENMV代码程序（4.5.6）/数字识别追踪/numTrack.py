import sensor, image, time,math,os, tf, math, uos, gc
from pyb import Pin,Timer,UART
import ustruct

class NumTrack():
    def __init__(self):
        # 初始化摄像头参数
        self.init_camera()
        
        # 加载模型
        self.load_model()
        
        # 初始化串口和定时器
        self.uart = UART(3,115200)
        self.uart.init(115200, bits=8, parity=None, stop=1)
        
        self.tim = Timer(4, freq=1000)
        self.led_dac = self.tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
        self.led_dac.pulse_width_percent(50)
        
        # 初始化位置和控制参数
        self.mid_block_cx = 80.5
        self.mid_block_cy = 60.5
        self.servo0 = 1500
        self.servo1 = 1500
        
        # 发送初始位置指令
        self.uart.write("{{#000P{:0>4d}T1100!#001P{:0>4d}T1100!}}\n".format(self.servo0, self.servo1))
        
        # 初始化追踪状态
        self.cap_num_status = 3
        self.detection_history = []  # 用于存储最近的检测结果
        
        # 数字识别优化参数
        self.min_confidence = 0.7  # 初始置信度阈值
        self.confidence_threshold = 0.7  # 最终识别的置信度阈值
        self.history_size = 5  # 历史记录大小
        self.consensus_threshold = 3  # 达成共识的阈值
        
        # 颜色定义
        self.colors = [
            (255,   0,   0),
            (  0, 255,   0),
            (255, 255,   0),
            (  0,   0, 255),
            (255,   0, 255),
            (  0, 255, 255),
            (255, 255, 255),
        ]
    
    def init_camera(self):
        sensor.reset()
        sensor.set_pixformat(sensor.GRAYSCALE)
        sensor.set_framesize(sensor.QQVGA)
        sensor.skip_frames(n=2000)
        
        # 关闭自动增益和白平衡以提高一致性
        sensor.set_auto_gain(False, value=10)  # 固定增益值
        sensor.set_auto_whitebal(False, rgb_gain_db=(0,0,0))  # 固定白平衡
        
        # 增加对比度和锐度
        sensor.set_contrast(3)  # 对比度设置为最高
        sensor.set_brightness(0)  # 亮度设置为中性
        sensor.set_saturation(0)  # 饱和度设置为中性
        sensor.set_sharpness(3)  # 锐度设置为最高
    
    def load_model(self):
        # 尝试加载模型，并根据可用内存大小确定是否将模型文件加载到堆上
        try:
            self.net = tf.load("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
            # 加载标签文件
            try:
                with open('labels.txt', 'r') as f:
                    self.labels = [line.rstrip() for line in f.readlines()]
            except:
                self.labels = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
        except Exception as e:
            raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')
    
    def init(self, cx=80.5, cy=60.5):
        self.init_camera()
        self.uart.init(115200, bits=8, parity=None, stop=1)
        self.led_dac.pulse_width_percent(10)
        
        self.mid_block_cx = cx
        self.mid_block_cy = cy
        self.cap_num_status = 3
        
        # 重置历史记录
        self.detection_history = []
        
        time.sleep_ms(1000)
    
    def update_detection_history(self, detection):
        # 更新检测历史记录
        self.detection_history.append(detection)
        if len(self.detection_history) > self.history_size:
            self.detection_history.pop(0)
    
    def get_consensus_detection(self):
        # 如果历史记录为空，返回None
        if not self.detection_history:
            return None
        
        # 统计每个检测结果出现的次数
        detection_counts = {}
        for detection in self.detection_history:
            if detection is not None:
                key = (detection[0], detection[1])  # (class_id, confidence)
                if key in detection_counts:
                    detection_counts[key] += 1
                else:
                    detection_counts[key] = 1
        
        # 如果没有足够的检测结果，返回None
        if not detection_counts:
            return None
        
        # 找出出现次数最多的检测结果
        most_common = max(detection_counts.items(), key=lambda x: x[1])
        
        # 如果出现次数超过阈值，返回该结果
        if most_common[1] >= self.consensus_threshold:
            return most_common[0]
        
        return None
    
    def preprocess_image(self, img):
        # 图像预处理以提高识别精度
        # 直方图均衡化增强对比度
        img.histeq()
        
        # 应用高斯模糊减少噪声
        img.gaussian(1)
        
        # 应用锐化滤镜增强边缘
        img.sharpen(1)
        
        return img
    
    def run(self):
        # 物块中心点
        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy
        detected_class = None
        highest_confidence = 0
        
        # 获取并预处理图像
        img = sensor.snapshot()
        img = self.preprocess_image(img)
        
        # 检测数字
        for i, detection_list in enumerate(self.net.detect(img, thresholds=[(math.ceil(self.min_confidence * 255), 255)])):
            if detection_list is None:
                continue
            if i == 0:  # 跳过背景类
                continue
            if len(detection_list) == 0:  # 没有检测到该类别的对象
                continue
            
            for d in detection_list:
                [x, y, w, h] = d.rect()
                confidence = d.output()[i]
                
                # 只考虑指定的数字类别
                if i == self.cap_num_status and confidence > highest_confidence:
                    highest_confidence = confidence
                    block_cx = x + w//2  # 计算中心点x坐标
                    block_cy = y + h//2  # 计算中心点y坐标
                    detected_class = i
        
        # 更新检测历史记录
        if detected_class is not None:
            self.update_detection_history((detected_class, highest_confidence))
        else:
            self.update_detection_history(None)
        
        # 获取共识检测结果
        consensus = self.get_consensus_detection()
        
        # 只有当达成共识且置信度足够高时才更新位置
        if consensus and consensus[1] >= self.confidence_threshold:
            detected_class, confidence = consensus
            # 绘制检测结果
            img.draw_rectangle(d.rect(), color=self.colors[detected_class % len(self.colors)])
            img.draw_cross(int(block_cx), int(block_cy), size=5, color=(255,0,0))
            img.draw_string(d[0], (d[1]-10), str(detected_class), color=(255,255,255))
            
            # 计算移动量 - 使用PID控制逻辑
            move_x = self.calculate_movement(block_cx, 80, 0.8, 0.1, 0.05)
            move_y = self.calculate_movement(block_cy, 60, 1.0, 0.15, 0.07)
            
            # 更新舵机位置
            self.servo0 = int(self.servo0 + move_x)
            self.servo1 = int(self.servo1 + move_y)
        else:
            # 没有检测到或置信度不够，逐渐回到中心位置
            self.servo0 = int(self.servo0 + (1500 - self.servo0) * 0.1)
            self.servo1 = int(self.servo1 + (1500 - self.servo1) * 0.1)
        
        # 限制舵机范围
        self.servo0 = max(650, min(2400, self.servo0))
        self.servo1 = max(500, min(2400, self.servo1))
        
        # 发送舵机控制指令
        self.uart.write("{{#000P{:0>4d}T0000!#001P{:0>4d}T0000!}}\n".format(self.servo0, self.servo1))
        time.sleep_ms(50)
    
    def calculate_movement(self, current, target, kp, ki, kd):
        # 简单的PID控制器实现
        error = target - current
        self.prev_error = getattr(self, 'prev_error', 0)
        self.integral = getattr(self, 'integral', 0)
        
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        
        # 防止积分饱和
        self.integral = max(-50, min(50, self.integral))
        
        # 计算输出
        output = kp * error + ki * self.integral + kd * derivative
        return output
