import sensor, time,math
from pyb import Pin,Timer,UART

class ColorPalletizer():
    red_threshold = (0, 100, 20, 127, 0, 127)
    blue_threshold = (0, 100, -128, 127, -128, -15)
    green_threshold = (0, 49, -22, -39, 74, 2)
    yellow_threshold = (57, 100, -33, 70, 48, 127)

    uart = UART(3,115200)   #设置串口波特率，与stm32一致
    uart.init(115200, bits=8, parity=None, stop=1 )

    tim = Timer(4, freq=1000) # Frequency in Hz
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=0)

    #物块中心点
    mid_block_cx=80
    mid_block_cy=60

    move_status=0#运行状态标记

    #机械臂移动位置
    move_y=0
    move_x=150

    mid_block_cnt=0#机械臂移到色块中心计数
    palletizer_cnt=0#记录码垛的数量

    def init(self):#初始化
        sensor.reset() #初始化摄像头
        sensor.set_pixformat(sensor.RGB565) #图像格式为 RGB565 灰度 GRAYSCALE
        sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
        sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
        sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
        sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡

        self.uart.init(115200, bits=8, parity=None, stop=1 )
        self.led_dac.pulse_width_percent(100)

        self.move_status=0
        self.mid_block_cnt=0
        self.palletizer_cnt=0

        #机械臂移动位置
        self.move_y=0
        self.move_x=150

        self.uart.write("$KMS:{:03d},{:03d},{:03d},1000!\n".format(int(self.move_x), int(self.move_y), 120))
        time.sleep_ms(1000)

    def run(self,cx=0,cy=0,cz=0):#运行功能
        '''
            3个变量控制机械臂抓取色块时的偏移量,如果机械臂抓取色块失败则调整变量
            cx: 偏前减小, 偏后增加
            cy: 偏右减小，偏左增加
            cz: 偏高减小，偏低增加
        '''

        #物块中心点
        block_cx=self.mid_block_cx
        block_cy=self.mid_block_cy

        color_read_succed=0#是否识别到颜色
        # 获取图像
        img = sensor.snapshot()
        red_blobs = img.find_blobs([self.red_threshold],x_stride=15, y_stride=15, pixels_threshold=25 )
        blue_blobs = img.find_blobs([self.blue_threshold], x_stride=15, y_stride=15, pixels_threshold=25 )
        green_blobs = img.find_blobs([self.green_threshold], x_stride=15, y_stride=15, pixels_threshold=25 )
        #***************首先进行色块检测********************
        if red_blobs:#红色
            color_read_succed=1
            for y in red_blobs:
                img.draw_rectangle((y[0],y[1],y[2],y[3]),color=(255,255,255))
                img.draw_cross(y[5], y[6],size=2,color=(255,0,0))
                img.draw_string(y[0], (y[1]-10), "red", color=(255,0,0))
                #print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","红色")
                block_cx=y[5]
                block_cy=y[6]

        elif blue_blobs:#蓝色
            color_read_succed=1
            for y in blue_blobs:
                img.draw_rectangle((y[0],y[1],y[2],y[3]),color=(255,255,255))
                img.draw_cross(y[5], y[6],size=2,color=(0,0,255))
                img.draw_string(y[0], (y[1]-10), "blue", color=(0,0,255))
                #print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","蓝色")
                block_cx=y[5]
                block_cy=y[6]

        elif green_blobs:#绿色
            color_read_succed=1
            for y in green_blobs:
                img.draw_rectangle((y[0],y[1],y[2],y[3]),color=(255,255,255))
                img.draw_cross(y[5], y[6],size=2,color=(0,255,0))
                img.draw_string(y[0], (y[1]-10), "green", color=(0,255,0))
                #print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","绿色")
                block_cx=y[5]
                block_cy=y[6]

        #************************************************ 运动机械臂*************************************************************************************
        if color_read_succed==1 or (self.move_status==1):#识别到颜色
            if self.move_status==0:#第0阶段：机械臂寻找物块位置
                if(abs(block_cx-self.mid_block_cx)>3):
                    if block_cx > self.mid_block_cx:
                        self.move_y+=0.2
                    else:
                        self.move_y-=0.2
                if(abs(block_cy-self.mid_block_cy)>3):
                    if block_cy > self.mid_block_cy and self.move_x>80:
                        self.move_x-=0.3
                    else:
                        self.move_x+=0.3
                if abs(block_cy-self.mid_block_cy)<=3 and abs(block_cx-self.mid_block_cx)<=3: #寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt>10:#计数10次对准物块，防止误差
                        self.mid_block_cnt=0
                        self.move_status=1
                else:
                    self.mid_block_cnt=0
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 120, 10))
                time.sleep_ms(10)

            elif self.move_status==1:#第1阶段：机械臂抓取物块
                l=math.sqrt(self.move_y*self.move_y+self.move_x*self.move_x)
                sin=self.move_x/l
                cos=self.move_y/l
                self.move_y=(l+85+cy)*cos+cx
                self.move_x=(l+85+cy)*sin
                time.sleep_ms(100)
                #移动机械臂到物块上方
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x) - 25, -(int(self.move_y)), 120, 1000))
                time.sleep_ms(100)
                self.uart.write("{#005P1300T1000!}")#张开爪子
                time.sleep_ms(1000)
                #移动机械臂下移到物块
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x) - 25 - 20, -(int(self.move_y)) + 10, 15+cz, 1000))
                time.sleep_ms(1200)
                self.uart.write("{#005P1750T1000!}")#机械爪抓取物块
                time.sleep_ms(1200)
                #移动机械臂抬起
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 120, 1000))
                time.sleep_ms(1200)
                #机械臂旋转到要方向物块的指定位置
                self.move_y=-190
                self.move_x=20
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 120, 1000))
                time.sleep_ms(1200)
                if self.palletizer_cnt==0:#第1次码垛
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 15+cz, 1000))
                elif self.palletizer_cnt==1:#第2次码垛
                    #需要根据实际调整数据
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x)-5, -(int(self.move_y)), 15+cz+30, 1000))
                elif self.palletizer_cnt==2:#第3次码垛
                    #需要根据实际调整数据
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 15+cz+30+30, 1000))
                time.sleep_ms(1200)
                self.uart.write("{#005P1300T1000!}")#张开爪子
                time.sleep_ms(1200)
                #移动机械臂抬起
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 120, 1000))
                time.sleep_ms(1200)
                self.move_y=0#机械臂归位
                self.move_x=150
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 120, 1000))
                time.sleep_ms(1200)
                self.move_status=0
                self.palletizer_cnt=self.palletizer_cnt+1


if __name__ == "__main__":
    colorPalletizer=ColorPalletizer()
    colorPalletizer.init()#初始化

    while(1):
        colorPalletizer.run(0,0,0)#运行功能








