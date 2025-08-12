# 颜色追踪时需要控制环境光线稳定，避免识别标志物的色彩阈值发生改变
import sensor, image, time,math
from pyb import UART, Pin,Timer

red_threshold=(0, 100, 20, 127, 0, 127)
blue_threshold=(0, 100, -128, 127, -128, -15)
green_threshold=(0, 100, -128, -23, 16, 76)
yellow_threshold=(57, 100, -33, 70, 48, 127)

color_threshold = red_threshold#识别的颜色

sensor.reset() #初始化摄像头
sensor.set_pixformat(sensor.RGB565) #图像格式为 RGB565
sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡
clock = time.clock() #追踪帧率

uart = UART(3,115200)   #设置串口波特率，与stm32一致
uart.init(115200, bits=8, parity=None, stop=1 )

tim = Timer(4, freq=1000) # Frequency in Hz
led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
led_dac.pulse_width_percent(100)
led_status=0
action_demo=0
servo0=1500
servo1=1500
uart.write("{{#000P{:0>4d}T1100!#001P{:0>4d}T1100!}}\n".format(servo0,servo1))

servo_option=1#操作舵机选择

def init():
    global clock,uart,tim,led_dac,led_status,move_x,move_y
    global servo0,servo1,servo_option,action_demo
    sensor.reset() #初始化摄像头
    sensor.set_pixformat(sensor.RGB565) #图像格式为 RGB565
    sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
    sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
    sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
    sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡
    clock = time.clock() #追踪帧率

    uart = UART(3,115200)   #设置串口波特率，与stm32一致
    uart.init(115200, bits=8, parity=None, stop=1 )

    tim = Timer(4, freq=1000) # Frequency in Hz
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
    led_dac.pulse_width_percent(100)
    led_status=0

    servo0=1500
    servo1=1500
    uart.write("{{#000P{:0>4d}T1100!#001P{:0>4d}T1100!}}\n".format(servo0,servo1))
    action_demo=0
    servo_option=1#操作舵机选择

    time.sleep_ms(2000)

def run():
    global color_threshold,clock,uart,tim,led_dac,led_status,move_x,move_y
    global servo0,servo1,servo_option,action_demo
    action_demo+=1#演示动作标记
    if action_demo>100: action_demo=100

    # 拍摄一张照片
    img = sensor.snapshot()
    blobs = img.find_blobs([color_threshold],x_stride=20, y_stride=20, pixels_threshold=36 )
    if blobs:
        #演示动作
        if action_demo>=50:

            for i in range(5):
                uart.write("#005P1000T0500!}\n")
                time.sleep_ms(100)
            for i in range(5):
                uart.write("#005P1500T0500!}\n")
                time.sleep_ms(100)
        action_demo=0

        max_size = 0
        max_blob=blobs[0]
        for blob in blobs:#寻找最大
            if blob[2] * blob[3] > max_size:
                max_blob = blob
                max_size = blob[2] * blob[3]
        img.draw_rectangle((max_blob[0],max_blob[1],max_blob[2],max_blob[3]),color=(255,255,255))
        img.draw_cross(max_blob[5], max_blob[6],size=2,color=(255,0,0))
        img.draw_string(max_blob[0], (max_blob[1]-10), "red", color=(255,0,0))
        #print("中心X坐标",max_blob[5],"中心Y坐标",max_blob[6],"识别颜色类型","红色")
        block_cx=max_blob[5]
        block_cy=max_blob[6]

        #************************运动机械臂**********************************
        if(abs(block_cx-80)>=5):
            if block_cx > 80:
                move_x=-0.8*abs(block_cx-80)
            else:
                move_x=0.8*abs(block_cx-80)
            servo0=int(servo0+move_x)

        if(abs(block_cy-60)>=2):
            if block_cy > 60:
                move_y=-1*abs(block_cy-60)
            else:
                move_y=1*abs(block_cy-60)
            servo1=int(servo1+move_y)


        if servo0>2400: servo0=2400
        elif servo0<650: servo0=650
        if servo1>2400: servo1=2400
        elif servo1<500: servo1=500

        uart.write("{{#000P{:0>4d}T0000!#001P{:0>4d}T0000!}}\n".format(servo0,servo1))
        time.sleep_ms(50)


