import sensor, image, time, ml, math, uos, gc
from pyb import Pin,Timer,UART

class ApriltagNumSort():
    net = None
    labels = None
    min_confidence = 0.5

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
    led_dac.pulse_width_percent(0)

    cap_num_status=0#抓取物块颜色标志，用来判断物块抓取
    #机械臂移动位置
    move_y=0
    move_x=150

    mid_block_cx=80
    mid_block_cy=60

    mid_block_cnt=0#用来记录机械臂已对准物块计数，防止误差

    move_status=0#机械臂移动的方式

    #用来记录已经抓取到标签
    apriltag_succeed_flag=0

    block_degress=0#机械爪旋转角度

    cap_num_status=0#抓取物块颜色标志，用来判断物块抓取的顺序

    def init(self):
        sensor.reset() #初始化摄像头
        sensor.set_pixformat(sensor.GRAYSCALE) #图像格式为 RGB565 灰度 GRAYSCALE
        sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
        sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
        sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
        sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡

        self.uart.init(115200, bits=8, parity=None, stop=1 )
        self.led_dac.pulse_width_percent(0)

        self.cap_num_status=0#抓取物块标签标志，用来判断物块抓取
        #机械臂移动位置
        self.move_y=0
        self.move_x=150

        self.mid_block_cnt=0#用来记录机械臂已对准物块计数，防止误差

        self.move_status=0#机械臂移动的方式

        #用来记录已经抓取到标签
        self.apriltag_succeed_flag=0

        self.block_degress=0#机械爪旋转角度

        self.uart.write("$KMS:{:03d},{:03d},{:03d},1000!\n".format(int(self.move_x), int(self.move_y), 120))
        time.sleep_ms(1000)

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

    def run(self,cx=0,cy=0,cz=0):#运行功能
        '''
            3个变量控制机械臂抓取色块时的偏移量,如果机械臂抓取色块失败则调整变量
            cx: 偏右减小, 偏左增加
            cy: 偏前减小，偏后增加
            cz: 偏高减小，偏低增加
        '''
        #物块中心点
        block_cx=self.mid_block_cx
        block_cy=self.mid_block_cy
        block_read_succed=0#是否识别到
        # 获取图像
        img = sensor.snapshot()

        if self.apriltag_succeed_flag==0:#识别抓取标签

            for tag in img.find_apriltags(): # defaults to TAG36H11 without "families".
                img.draw_rectangle(tag.rect, color = (255, 0, 0))     #画框
                img.draw_cross(tag.cx, tag.cy, color = (0, 255, 0)) #画十字
                img.draw_string(tag.x, (tag.y-10), "{}".format(tag.id), color=(255,0,0))
                block_cx=tag.cx
                block_cy=tag.cy
                self.block_degress = 180 * tag.rotation / math.pi                #求April Tags旋转的角度
                self.cap_num_status=tag.id
                block_read_succed=1

        elif self.apriltag_succeed_flag==1:#抓取到标签后数字分拣

            for i, detection_list in enumerate(self.net.predict([img], callback=self.fomo_post_process)):
                if (i == 0): continue # background class
                if (len(detection_list) == 0): continue # no detections for this class?
                for x, y, w, h, score in detection_list:
                    if i==self.cap_num_status:
                        block_cx = x
                        block_cy = y
                        block_read_succed=1
                        #print(" %s " % labels[i],block_cx,block_cy)
                        img.draw_rectangle(x, y, w, h,color=(255,255,255))
                        img.draw_cross(block_cx,block_cy,size=2,color=(255,0,0))
                        img.draw_string(x, y-10, self.labels[i], color=(255,255,255))

        #************************************************ 运动机械臂*************************************************************************************
        if block_read_succed==1 or (self.move_status==1):#识别到颜色或者到路口

            if self.move_status==0:#第0阶段：机械臂寻找物块位置
                if(abs(block_cx-self.mid_block_cx)>2):
                    if block_cx > self.mid_block_cx:
                        self.move_y+=0.2
                    else:
                        self.move_y-=0.2
                if(abs(block_cy-self.mid_block_cy)>2):
                    if block_cy > self.mid_block_cy and self.move_x>80:
                        self.move_x-=0.3
                    else:
                        self.move_x+=0.3
                if abs(block_cy-self.mid_block_cy)<=2 and abs(block_cx-self.mid_block_cx)<=2: #寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt>10:#计数10次对准物块，防止误差
                        self.mid_block_cnt=0
                        self.move_status=1
                else:
                    self.mid_block_cnt=0
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 120, 10))
                time.sleep_ms(10)

            elif self.move_status==1:#第1阶段：机械臂抓取物块
                self.move_status=2
                time.sleep_ms(100)
                # 判断矩形倾角，改变机械爪
                spin_calw = 1500
                if self.block_degress % 90 < 45:
                    spin_calw = int(1500 - self.block_degress % 90 * 500 / 90)
                else:
                    spin_calw = int((90 - self.block_degress % 90) * 500 / 90 + 1500)

                if spin_calw >= 2500 or spin_calw <= 500:
                    spin_calw = 1500
                self.uart.write("{{#004P{:0^4}T1000!}}".format(spin_calw))#旋转和张开机械爪
                l=math.sqrt(self.move_y*self.move_y+self.move_x*self.move_x)
                sin=self.move_x/l
                cos=self.move_y/l
                self.move_y=(l+85+cy)*cos+cx
                self.move_x=(l+85+cy)*sin
                time.sleep_ms(100)
                #移动机械臂到物块上方
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x)  - 25, -(int(self.move_y)) + 15, 120, 1000))
                time.sleep_ms(100)
                self.uart.write("{#005P1100T1000!}")
                time.sleep_ms(1000)
                #移动机械臂下移到物块
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x)  - 25, -(int(self.move_y)) + 15, 5+cz, 1000))
                time.sleep_ms(1200)
                self.uart.write("{#005P1650T1000!}")#机械爪抓取物块
                time.sleep_ms(1200)
                #移动机械臂抬起
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 120, 1000))
                self.uart.write("{#004P1500T1000!}")#旋转和张开机械爪
                time.sleep_ms(1200)
                #机械臂旋转到要方向物块的指定位置
                if self.cap_num_status==1:
                    self.move_y=-150
                    self.move_x=-30
                elif self.cap_num_status==2:
                    self.move_y=-150
                    self.move_x=30
                elif self.cap_num_status==3:
                    self.move_y=-130
                    self.move_x=60
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 120, 1000))
                time.sleep_ms(1200)
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 50, 1000))
                time.sleep_ms(1200)
                self.mid_block_cnt=0
                self.apriltag_succeed_flag=1#抓取到标签后数字分拣

            elif self.move_status==2:#第2阶段：机械臂寻找放下物块的框框
                if(abs(block_cx-self.mid_block_cx)>5):
                    if block_cx > self.mid_block_cx and self.move_x>1:
                        self.move_x-=0.3
                    else:
                        self.move_x+=0.3
                if(abs(block_cy-self.mid_block_cy)>5):
                    if block_cy > self.mid_block_cy:
                        self.move_y+=0.2
                    else:
                        self.move_y-=0.2

                if abs(block_cy-self.mid_block_cy)<=5 and abs(block_cx-self.mid_block_cx)<=5: #寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt>5:#计数5次对准物块，防止误差
                        self.mid_block_cnt=0
                        self.move_status=3
                else:
                    self.mid_block_cnt=0
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 70, 10))
                time.sleep_ms(10)

            elif self.move_status==3:#第3阶段：机械臂放下物块并归位
                self.move_status=0
                time.sleep_ms(100)
                l=math.sqrt(self.move_y*self.move_y+self.move_x*self.move_x)
                sin=self.move_x/l
                cos=self.move_y/l
                self.move_y=(l+85+cy)*cos
                self.move_x=(l+85+cy)*sin
                #移动机械臂到物块上方
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x) - 10, -(int(self.move_y)) - 40, 70, 1000))
                time.sleep_ms(1200)
                #移动机械臂下移到物块
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x) - 10, -(int(self.move_y)) - 40, 5+cz, 1000))
                time.sleep_ms(1200)
                self.uart.write("{#005P1100T1000!}")#机械爪放下物块
                time.sleep_ms(1200)
                #移动机械臂抬起
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 120, 1000))
                time.sleep_ms(1200)
                self.move_y=0#机械臂归位
                self.move_x=150
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n".format(int(self.move_x), -(int(self.move_y)), 120, 1000))
                time.sleep_ms(1200)
                self.mid_block_cnt=0
                self.cap_num_status=0
                self.apriltag_succeed_flag=0#识别抓取标签


if __name__ == "__main__":
    app=ApriltagNumSort()
    app.init()#初始化

    while(1):
        app.run()#运行功能

