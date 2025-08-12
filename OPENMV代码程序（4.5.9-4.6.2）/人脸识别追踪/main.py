from pyb import UART, Pin,Timer
import sensor,time
import faceTrack

face_track = faceTrack.FaceTrack()

sensor.reset() #初始化摄像头
sensor.set_pixformat(sensor.RGB565) #图像格式为 RGB565
sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡
clock = time.clock() #追踪帧率

uart = UART(3,115200)   #设置串口波特率，与stm32一致
uart.init(115200, bits=8, parity=None, stop=1 )
time.sleep_ms(1000)
uart.write("{{#000P1500T1100!#001P1500T1000!}}\n")

tim = Timer(4, freq=1000) # Frequency in Hz
led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
led_dac.pulse_width_percent(0)
led_status=0

run_app_status=0

def beep():
    for i in range(2):
        uart.write("$BEEP!\n")#发送蜂鸣器鸣叫指令
        time.sleep_ms(200)
face_track.init()
while(True):
    face_track.run_track()#运行人脸追踪功能
    if uart.any():#接收指令
        try:#用来判断串口数据异常
            string = uart.read()
            if string:
                string = string.decode()
                # print(string)
                if string.find("#StartLed!") >= 0 :#开灯指令
                    led_dac.pulse_width_percent(100)
                    beep()
                elif string.find("#StopLed!") >= 0 :#关灯指令
                    led_dac.pulse_width_percent(0)
                    beep()
        except Exception as e:#串口数据异常进入
            print('Error:', e)







