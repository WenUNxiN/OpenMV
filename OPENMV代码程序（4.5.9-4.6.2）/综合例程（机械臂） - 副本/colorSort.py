import sensor,time,math
from pyb import Pin,Timer,UART
import kinematics

class ColorSort():
    red_threshold = (0, 100, 20, 127, 0, 127)
    blue_threshold = (0, 100, -128, 127, -128, -15)
    green_threshold = (0, 100, -128, -28, 0, 70)
    yellow_threshold = (57, 100, -33, 70, 48, 127)

    uart = UART(3,115200)   #设置串口波特率，与stm32一致
    uart.init(115200, bits=8, parity=None, stop=1 )

    tim = Timer(4, freq=1000) # Frequency in Hz
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
    led_dac.pulse_width_percent(100)

    cap_color_status=0#抓取物块颜色标志，用来判断物块抓取
    #机械臂移动位置
    move_y=0
    move_x=100

    #摄像头中点
    mid_block_cx=80
    mid_block_cy=60

    mid_block_cnt=0#用来记录机械臂已对准物块计数，防止误差
    move_status=0#机械臂移动的方式


    def init(self):#初始化
        sensor.reset() #初始化摄像头
        sensor.set_pixformat(sensor.RGB565) #图像格式为 RGB565 灰度 GRAYSCALE
        sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
        sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
        sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
        sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡

        self.cap_color_status=0#抓取物块颜色标志，用来判断物块抓取
        #机械臂移动位置
        self.move_y=0
        self.move_x=150

        self.mid_block_cnt=0#用来记录机械臂已对准物块计数，防止误差
        self.move_status=0#机械臂移动的方式

        self.led_dac.pulse_width_percent(100)
        self.uart.write("$KMS:{:03d},{:03d},{:03d},1000!\n".format(int(self.move_x), int(self.move_y), 150))
        time.sleep_ms(1000)

    def run(self,cx=0,cy=0,cz=30):#运行功能
        '''
            3个变量控制机械臂抓取色块时的偏移量,如果机械臂抓取色块失败则调整变量
            cx: 偏右减小, 偏左增加
            cy: 偏前减小，偏后增加
            cz: 偏高减小，偏低增加
        '''
        #物块中心点
        block_cx=self.mid_block_cx
        block_cy=self.mid_block_cy
        color_read_succed=0#是否识别到颜色
        color_status=0
        # 获取图像
        img = sensor.snapshot()
        red_blobs = img.find_blobs([self.red_threshold],x_stride=15, y_stride=15, pixels_threshold=25 )
        blue_blobs = img.find_blobs([self.blue_threshold], x_stride=15, y_stride=15, pixels_threshold=25 )
        green_blobs = img.find_blobs([self.green_threshold], x_stride=15, y_stride=15, pixels_threshold=50 )
        #***************首先进行色块检测********************
        if red_blobs and (self.cap_color_status==0 or self.cap_color_status=='R'):#红色
            color_status = 'R'
            color_read_succed=1
            for y in red_blobs:
                img.draw_rectangle((y[0],y[1],y[2],y[3]),color=(255,255,255))
                img.draw_cross(y[5], y[6],size=2,color=(255,0,0))
                img.draw_string(y[0], (y[1]-10), "red", color=(255,0,0))
                #print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","红色")
                block_cx=y[5]
                block_cy=y[6]

        elif blue_blobs and (self.cap_color_status==0 or self.cap_color_status=='B'):#蓝色
            color_status = 'B'
            color_read_succed=1
            for y in blue_blobs:
                img.draw_rectangle((y[0],y[1],y[2],y[3]),color=(255,255,255))
                img.draw_cross(y[5], y[6],size=2,color=(0,0,255))
                img.draw_string(y[0], (y[1]-10), "blue", color=(0,0,255))
                #print("中心X坐标",y[5],"中心Y坐标",y[6],"识别颜色类型","蓝色")
                block_cx=y[5]
                block_cy=y[6]

        elif green_blobs and (self.cap_color_status==0 or self.cap_color_status=='G'):#绿色
            color_status = 'G'
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
                    if self.mid_block_cnt > 10:#计数10次对准物块，防止误差
                        self.mid_block_cnt=0
                        self.move_status=1
                        self.cap_color_status=color_status
                else:
                    self.mid_block_cnt=0
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 150, 10))
                time.sleep_ms(100)

            elif self.move_status==1:#第1阶段：机械臂抓取物块
                self.move_status=2
                time.sleep_ms(100)
                self.uart.write("{#005P1100T1000!}")
                l=math.sqrt(self.move_y*self.move_y+self.move_x*self.move_x)
                sin=self.move_x/l
                cos=self.move_y/l
                self.move_y=(l+85+cy)*cos+cx
                self.move_x=(l+85+cy)*sin
                time.sleep_ms(100)
                #移动机械臂到物块上方
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x)  - 30, -(int(self.move_y)) + 10, 150, 1000))
                time.sleep_ms(1000)
                #移动机械臂下移到物块
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x)  - 30, -(int(self.move_y)) + 10, 25+cz, 1000))
                time.sleep_ms(1200)
                self.uart.write("{#005P1650T1000!}")#机械爪抓取物块
                time.sleep_ms(1200)
                #移动机械臂抬起
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 150, 1000))
                time.sleep_ms(1200)
                #机械臂旋转到要方向物块的指定位置
                self.move_y=100
                self.move_x=60
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 150, 1000))
                time.sleep_ms(1200)
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 150, 1000))
                time.sleep_ms(1200)
                self.mid_block_cnt=0

            elif self.move_status==2:#第2阶段：机械臂寻找放下物块的框框
                if(abs(block_cx-self.mid_block_cx)>3):
                    if block_cx > self.mid_block_cx and self.move_x>1:
                        self.move_x-=0.3
                    else:
                        self.move_x+=0.3
                if(abs(block_cy-self.mid_block_cy)>3):
                    if block_cy > self.mid_block_cy:
                        self.move_y-=0.2
                    else:
                        self.move_y+=0.2
                if abs(block_cy-self.mid_block_cy)<=3 and abs(block_cx-self.mid_block_cx)<=3: #寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt>5:#计数5次对准物块，防止误差
                        self.mid_block_cnt=0
                        self.move_status=3
                        self.cap_color_status=color_status
                else:
                    self.mid_block_cnt=0
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 150, 10))
                time.sleep_ms(10)

            elif self.move_status==3:#第3阶段：机械臂放下物块并归位
                self.move_status=0
                l=math.sqrt(self.move_y*self.move_y+self.move_x*self.move_x)
                sin=self.move_x/l
                cos=self.move_y/l
                self.move_y=(l+85+cy)*cos
                self.move_x=(l+85+cy)*sin
                time.sleep_ms(100)
                #移动机械臂到物块上方
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x) + 10, -(int(self.move_y)) + 50, 150, 1000))
                time.sleep_ms(1000)
                #移动机械臂下移到物块
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x) + 10, -(int(self.move_y)) + 50, 25+cz+30, 1000))
                time.sleep_ms(1200)
                self.uart.write("{#005P1100T1000!}")#机械爪放下物块
                time.sleep_ms(1200)
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 150, 1000))#移动机械臂抬起
                time.sleep_ms(1200)
                self.move_y=0#机械臂归位
                self.move_x=150
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 150, 1000))
                time.sleep_ms(1200)
                self.mid_block_cnt=0
                self.cap_color_status=0



if __name__ == "__main__":
    colorSort=ColorSort()
    colorSort.init()#初始化

    while(1):
        colorSort.run(0,0,0)#运行功能




