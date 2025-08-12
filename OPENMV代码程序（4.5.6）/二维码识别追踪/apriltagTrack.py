import sensor, image, time,math,os, tf, math, uos, gc
from pyb import Pin,Timer,UART
import ustruct

class ApriltagTrack():
    sensor.reset() #初始化摄像头
    sensor.set_pixformat(sensor.GRAYSCALE) #图像格式为 RGB565 灰度 GRAYSCALE
    sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
    sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
    sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
    sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡

    uart = UART(3,115200)   #设置串口波特率，与stm32一致
    uart.init(115200, bits=8, parity=None, stop=1 )

    tim = Timer(4, freq=1000) # Frequency in Hz
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
    led_dac.pulse_width_percent(50)

    mid_block_cx=80.5
    mid_block_cy=60.5

    servo0 = 1500
    servo1 = 1500
    #uart.write("{{#000P{:0>4d}T1100!#001P{:0>4d}T1100!}}\n".format(servo0, servo1))

    servo_option = 1  # 操作舵机选择

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

        #uart.write("{{#000P{:0>4d}T1100!#001P{:0>4d}T1100!}}\n".format(self.servo0, self.servo1))

        time.sleep_ms(2000)

    def run(self):#分拣
        #物块中心点
        block_cx=self.mid_block_cx
        block_cy=self.mid_block_cy

        # 获取图像
        img = sensor.snapshot()

        for tag in img.find_apriltags(): # defaults to TAG36H11 without "families".
            img.draw_rectangle(tag.rect(), color = (255, 0, 0))     #画框
            img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0)) #画十字
            img.draw_string(tag[0], (tag[1]-10), "{}".format(tag.id()), color=(255,0,0))
            block_cx=tag.cx()
            block_cy=tag.cy()


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





