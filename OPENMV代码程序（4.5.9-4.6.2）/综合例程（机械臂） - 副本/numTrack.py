import sensor, image, time,math,os, ml, math, uos, gc
from pyb import Pin,Timer,UART
import ustruct

class NumTrack():
    sensor.reset() #初始化摄像头
    sensor.set_pixformat(sensor.GRAYSCALE) #图像格式为 RGB565 灰度 GRAYSCALE
    sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
    sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
    sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
    sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡

    net = None
    labels = None
    min_confidence = 0.7

    try:
       # load the model, alloc the model file on the heap if we have at least 64K free after loading
       net = ml.Model("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
    except Exception as e:
       raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

    try:
       labels = [line.rstrip('\n') for line in open("labels.txt")]
    except Exception as e:
       raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

    colors = [ # Add more colors if you are detecting more than 7 types of classes at once.
        (255,   0,   0),
        (  0, 255,   0),
        (255, 255,   0),
        (  0,   0, 255),
        (255,   0, 255),
        (  0, 255, 255),
        (255, 255, 255),
    ]

    threshold_list = [(math.ceil(min_confidence * 255), 255)]

    uart = UART(3,115200)   #设置串口波特率，与stm32一致
    uart.init(115200, bits=8, parity=None, stop=1 )

    tim = Timer(4, freq=1000) # Frequency in Hz
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
    led_dac.pulse_width_percent(50)

    mid_block_cx=80.5
    mid_block_cy=60.5

    servo0 = 1500
    servo1 = 1500
    uart.write("{{#000P{:0>4d}T1100!#001P{:0>4d}T1100!}}\n".format(servo0, servo1))

    def fomo_post_process(self,model, inputs, outputs):
        ob, oh, ow, oc = model.output_shape[0]

        x_scale = inputs[0].roi[2] / ow
        y_scale = inputs[0].roi[3] / oh

        scale = min(x_scale, y_scale)

        x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
        y_offset = ((inputs[0].roi[3] - (ow * scale)) / 2) + inputs[0].roi[1]

        l = [[] for i in range(oc)]

        for i in range(oc):
            img = image.Image(outputs[0][0, :, :, i] * 255)
            blobs = img.find_blobs(
                self.threshold_list, x_stride=1, y_stride=1, area_threshold=1, pixels_threshold=1
            )
            for b in blobs:
                rect = b.rect()
                x, y, w, h = rect
                score = (
                    img.get_statistics(thresholds=self.threshold_list, roi=rect).l_mean() / 255.0
                )
                x = int((x * scale) + x_offset)
                y = int((y * scale) + y_offset)
                w = int(w * scale)
                h = int(h * scale)
                l[i].append((x, y, w, h, score))
        return l

    def init(self,cx=80.5,cy=60.5):#初始化巡线配置，传入两个参数调整中位值
        sensor.reset() #初始化摄像头
        sensor.set_pixformat(sensor.GRAYSCALE) #图像格式为 RGB565 灰度 GRAYSCALE
        sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
        sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
        sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
        sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡

        self.uart.init(115200, bits=8, parity=None, stop=1 )
        self.led_dac.pulse_width_percent(10)

        self.mid_block_cx=cx
        self.mid_block_cy=cy

        self.cap_num_status= 2 #追踪物块的序号

        time.sleep_ms(1000)

    def run(self):#追踪
        #物块中心点
        block_cx=self.mid_block_cx
        block_cy=self.mid_block_cy
        # 获取图像
        img = sensor.snapshot()

        for i, detection_list in enumerate(self.net.predict([img], callback=self.fomo_post_process)):
            if (i == 0): continue # background class
            if (len(detection_list) == 0): continue # no detections for this class?
            for x, y, w, h, score in detection_list:
                if i==self.cap_num_status:
                    block_cx = x
                    block_cy = y
                    #print(" %s " % labels[i],block_cx,block_cy)
                    img.draw_rectangle(x, y, w, h,color=(255,255,255))
                    img.draw_cross(block_cx,block_cy,size=2,color=(255,0,0))
                    img.draw_string(x, y-10, self.labels[i], color=(255,255,255))

        #************************运动舵机**********************************
        if(abs(block_cx-80)>=5):
            if block_cx > 80:
                move_x=-0.8*abs(block_cx-80)
            else:
                move_x=0.8*abs(block_cx-80)
            self.servo0=int(self.servo0+move_x)

        if(abs(block_cy-60)>=2):
            if block_cy > 60:
                move_y=-1*abs(block_cy-60)
            else:
                move_y=1*abs(block_cy-60)
            self.servo1=int(self.servo1+move_y)


        if self.servo0>2400: self.servo0=2400
        elif self.servo0<650: self.servo0=650
        if self.servo1>2400: self.servo1=2400
        elif self.servo1<500: self.servo1=500

        self.uart.write("{{#000P{:0>4d}T0000!#001P{:0>4d}T0000!}}\n".format(self.servo0,self.servo1))
        time.sleep_ms(50)



