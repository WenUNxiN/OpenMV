import sensor, time
from pyb import Pin,Timer,UART


class ColorTrace():
    red_threshold = (0, 100, 20, 127, 0, 127)
    blue_threshold = (0, 100, -128, 127, -128, -15)
    green_threshold = (0, 100, -128, -28, 0, 70)
    yellow_threshold = (57, 100, -33, 70, 48, 127)
    track_color_threshold = yellow_threshold#追踪的颜色

    uart = UART(3,115200)   #设置串口波特率，与stm32一致
    uart.init(115200, bits=8, parity=None, stop=1 )

    tim = Timer(4, freq=1000) # Frequency in Hz
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=0)

    #物块中心点
    mid_block_cx=80
    mid_block_cy=60

    #机械臂移动位置
    servo0=1500
    servo1=1500

    def init(self):
        sensor.reset() #初始化摄像头
        sensor.set_pixformat(sensor.RGB565) #图像格式为 RGB565 灰度 GRAYSCALE
        sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
        sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
        sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
        sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡

        self.led_dac.pulse_width_percent(100)

        #机械臂移动位置
        self.servo0 = 1500
        self.servo1 = 1500

        self.uart.write("{{#000P{:0>4d}T1100!#001P{:0>4d}T1100!#002P{:0>4d}T1100!#003P{:0>4d}T1100!}}\n".format(self.servo0,self.servo1,1800,860))
        time.sleep_ms(1000)

    def run(self):#运行功能
        #物块中心点
        block_cx=self.mid_block_cx
        block_cy=self.mid_block_cy
        # 获取图像
        img = sensor.snapshot()

        #***************进行色块检测********************
        blobs = img.find_blobs([self.track_color_threshold],x_stride=15, y_stride=15, pixels_threshold=25 )
        max_size = 0
        max_blob = 0
        for blob in blobs:#寻找最大
            if blob[2] * blob[3] > max_size:
                max_blob = blob
                max_size = blob[2] * blob[3]

        if blobs:#色块
            img.draw_rectangle((max_blob[0],max_blob[1],max_blob[2],max_blob[3]),color=(255,255,255))
            img.draw_cross(max_blob[5], max_blob[6],size=2,color=(255,0,0))
            img.draw_string(max_blob[0], (max_blob[1]-10), "color", color=(255,0,0))
            #print("中心X坐标",max_blob[5],"中心Y坐标",max_blob[6],"识别颜色类型","红色")
            block_cx=max_blob[5]
            block_cy=max_blob[6]

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









