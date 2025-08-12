import sensor, image, time,math,os, ml, math, uos, gc
from pyb import Pin,Timer,UART
import ustruct

class NumTrack:
    # 类级别初始化摄像头参数
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QQVGA)  # 160x120
    sensor.skip_frames(n=2000)
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)

    net = None
    min_confidence = 0.7

    colors = [
        (255,   0,   0),
        (  0, 255,   0),
        (255, 255,   0),
        (  0,   0, 255),
        (255,   0, 255),
        (  0, 255, 255),
        (255, 255, 255),
    ]
    
    threshold_list = [(math.ceil(min_confidence * 255), 255)]

    uart = UART(3, 115200)
    uart.init(115200, bits=8, parity=None, stop=1)

    tim = Timer(4, freq=1000)
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)

    mid_block_cx = 80.5
    mid_block_cy = 60.5

    servo0 = 1500
    servo1 = 1500
    uart.write("{{#000P{:0>4d}T1100!#001P{:0>4d}T1100!}}\n".format(servo0, servo1))

    # 加载模型
    try:
       # load the model, alloc the model file on the heap if we have at least 64K free after loading
       net = ml.Model("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
    except Exception as e:
       raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

    def init(self, cx=80.5, cy=60.5):
        sensor.reset()
        sensor.set_pixformat(sensor.GRAYSCALE)
        sensor.set_framesize(sensor.QQVGA)
        sensor.skip_frames(n=2000)
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)

        self.uart.init(115200, bits=8, parity=None, stop=1)
        self.led_dac.pulse_width_percent(10)

        self.mid_block_cx = cx
        self.mid_block_cy = cy
        self.cap_num_status = 1  # 追踪物块编号
        time.sleep_ms(1000)

    def run(self):
        img = sensor.snapshot()

        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy

        detections = self.net.detect(img, thresholds=[(math.ceil(self.min_confidence * 255), 255)])
        if not detections:
            return

        for i, detection_list in enumerate(detections):
            if i == 0 or not detection_list:  # 忽略背景类或空检测
                continue

            for d in detection_list:
                x, y, w, h = d.rect()
                if i == self.cap_num_status:
                    # 更新为矩形中心点
                    block_cx = x + w // 2
                    block_cy = y + h // 2

                    # 绘图显示
                    img.draw_rectangle((x, y, w, h), color=(255, 255, 255))
                    img.draw_cross(block_cx, block_cy, size=2, color=(255, 0, 0))
                    img.draw_string(x, y - 10, str(i), color=(255, 255, 255))

        # ************************运动舵机**********************************
        move_x = 0
        if abs(block_cx - 80) >= 5:
            move_x = -0.8 * (block_cx - 80)

        move_y = 0
        if abs(block_cy - 60) >= 2:
            move_y = -1 * (block_cy - 60)

        self.servo0 = int(self.servo0 + move_x)
        self.servo1 = int(self.servo1 + move_y)

        # 限制舵机范围
        self.servo0 = max(650, min(2400, self.servo0))
        self.servo1 = max(500, min(2400, self.servo1))

        # 发送指令
        self.uart.write("{{#000P{:0>4d}T0000!#001P{:0>4d}T0000!}}\n".format(self.servo0, self.servo1))
        time.sleep_ms(50)
