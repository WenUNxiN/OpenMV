import sensor, image, time,math
from pyb import Pin,Timer,UART

class FaceTrack():
    sensor.reset() #初始化摄像头
    sensor.set_pixformat(sensor.GRAYSCALE) #图像格式为 RGB565 灰度 GRAYSCALE
    sensor.set_framesize(sensor.HQVGA) #QQVGA: 240x160
    sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
    sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
    sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡

    # 加载Haar算子
    # 默认情况下，这将使用所有阶段，更低的satges更快，但不太准确。
    face_cascade = image.HaarCascade("frontalface", stages=10)
    #image.HaarCascade(path, stages=Auto)加载一个haar模型。haar模型是二进制文件，
    #这个模型如果是自定义的，则引号内为模型文件的路径；也可以使用内置的haar模型，
    #比如“frontalface” 人脸模型或者“eye”人眼模型。
    #stages值未传入时使用默认的stages。stages值设置的小一些可以加速匹配，但会降低准确率。
    print(face_cascade)

    uart = UART(3,115200)   #设置串口波特率，与stm32一致
    uart.init(115200, bits=8, parity=None, stop=1 )

    tim = Timer(4, freq=1000) # Frequency in Hz
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
    led_dac.pulse_width_percent(50)

    mid_block_cx=120
    mid_block_cy=80

    move_x = 0
    move_y = 0

    servo0=1500
    servo1=1500
    #uart.write("{{#000P{:0>4d}T1100!#001P{:0>4d}T1100!}}\n".format(servo0,servo1))

    servo_option=1#操作舵机选择

    def init(self,cx=80.5,cy=60.5):#初始化巡线配置，传入两个参数调整中位值
        sensor.reset() #初始化摄像头
        # HQVGA和灰度对于人脸识别效果最好
        sensor.set_pixformat(sensor.GRAYSCALE) #图像格式为 RGB565 灰度 GRAYSCALE
        sensor.set_framesize(sensor.HQVGA) #QQVGA: 240x160
        sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
        sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
        sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡

        self.uart.init(115200, bits=8, parity=None, stop=1)
        self.led_dac.pulse_width_percent(0)

        self.servo_option=1#操作舵机选择

        move_x = 0
        move_y = 0
        mid_block_cx=120
        mid_block_cy=80

        servo0=1500
        servo1=1500
        #uart.write("{{#000P{:0>4d}T1100!#001P{:0>4d}T1100!}}\n".format(servo0,servo1))

    def run_track(self):#追踪
        cx = self.mid_block_cx
        cy = self.mid_block_cy
        # 拍摄一张照片
        img = sensor.snapshot()

        # Find objects.
        # Note: Lower scale factor scales-down the image more and detects smaller objects.
        # Higher threshold results in a higher detection rate, with more false positives.
        faces = img.find_features(self.face_cascade, threshold=0.75, scale=1.2)
        #image.find_features(cascade, threshold=0.5, scale=1.5),thresholds越大，
        #匹配速度越快，错误率也会上升。scale可以缩放被匹配特征的大小。

        #在找到的目标上画框，标记出来
        if faces:
            #追踪人脸
            max_size = 0
            max_blob=faces[0]
            for blob in faces:#寻找最大人脸
                if blob[2] * blob[3] > max_size:
                    max_blob = blob
                    max_size = blob[2] * blob[3]
            face=max_blob

            cx = int(face[0] + face[2] / 2)
            cy = int(face[1] + face[3] / 2)
            img.draw_rectangle(face,color=(255,0,0))
            img.draw_cross(cx, cy)               # 在目标区域的中心点处画十字

            #************************运动机械臂**********************************
            if abs(cx - self.mid_block_cx)>=5:
                if cx > self.mid_block_cx:
                    self.move_x=-0.5*abs(cx-self.mid_block_cx)
                else:
                    self.move_x=0.5*abs(cx-self.mid_block_cx)


            if abs(cy - self.mid_block_cy)>=2:
                if cy > self.mid_block_cy:
                    self.move_y=-0.8*abs(cy-self.mid_block_cy)
                else:
                    self.move_y=0.58*abs(cy-self.mid_block_cy)

                if self.servo_option==1:
                    self.servo_option=2
                    self.servo1=int(self.servo1+self.move_y)
                elif self.servo_option==2:
                    self.servo_option=1
                    self.servo0=int(self.servo0+self.move_x)

            if self.servo0>2400: self.servo0=2400
            elif self.servo0<650: self.servo0=650
            if self.servo1>2400: self.servo1=2400
            elif self.servo1<500: self.servo1=500
            #print("move_x",self.move_x,"move_y",self.move_y,"self.servo0",self.servo0,"self.servo1",self.servo1)
            self.uart.write("{{#000P{:0>4d}T0000!#001P{:0>4d}T0000!}}\n".format(self.servo0,self.servo1))
            time.sleep_ms(10)








