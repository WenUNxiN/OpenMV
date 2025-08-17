from pyb import UART, Pin,Timer
import time, pyb
import apriltagNumSort,apriltagPalletizer,apriltagSort
import colorSort,colorPalletizer,colorTrace
import faceTrack,apriltagTrack,numTrack

apriltag_Num_Sort = apriltagNumSort.ApriltagNumSort()
apriltag_Palletizer = apriltagPalletizer.ApriltagPalletizer()
apriltag_Sort = apriltagSort.ApriltagSort()
color_Sort = colorSort.ColorSort()
color_Palletizer = colorPalletizer.ColorPalletizer()
color_Trace = colorTrace.ColorTrace()
face_Track = faceTrack.FaceTrack()
april_tag_track = apriltagTrack.ApriltagTrack()
num_track = numTrack.NumTrack()

led = pyb.LED(3)

uart = UART(3,115200)   #设置串口波特率，与stm32一致
uart.init(115200, bits=8, parity=None, stop=1 )
uart.write("{{#000P{:0>4d}T1100!#001P{:0>4d}T1100!#002P{:0>4d}T1100!#003P{:0>4d}T1100!#004P{:0>4d}T1100!#005P{:0>4d}T1100!}}\n".format(1500,1500,1500,1500,1500,1500))

tim = Timer(4, freq=1000) # Frequency in Hz
led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
led_dac.pulse_width_percent(0)

run_app_status=0

uart.write("#openmv reset\n\n!")

'''
    3个变量控制机械臂抓取色块时的偏移量,如果机械臂抓取色块失败则调整变量
    cx: 偏右减小, 偏左增加
    cy: 偏前减小，偏后增加
    cz: 偏高减小，偏低增加
'''
cx=0
cy=0
cz=0

def beep():
    uart.write("$BEEP!\n")#发送蜂鸣器鸣叫指令
    led.on()            #亮灯
    time.sleep_ms(100)  #延时150ms
    led.off()           #暗灯
    time.sleep_ms(100)

if __name__ == "__main__":
    while(True):
        if uart.any():#接收指令
            try:#用来判断串口数据异常
                string = uart.read()
                print(string,isinstance(string.decode(), str))
                if string:
                    string = string.decode()
                    #print(string,isinstance(string, str),string.find("#start_led!"))
                    if string.find("#StartLed!") >= 0 :#开灯指令
                        led_dac.pulse_width_percent(100)
                        beep()
                    elif string.find("#StopLed!") >= 0 :#关灯指令
                        led_dac.pulse_width_percent(0)
                        beep()
                    elif string.find("#RunStop!") >= 0 :#停止所有运行并复位
                        run_app_status=0
                        led_dac.pulse_width_percent(0)
                        beep()
                    elif string.find("#ColorSort!") >= 0 :#色块分拣
                        run_app_status=1
                        color_Sort.init()
                        beep()
                    elif string.find("#ColorStack!") >= 0 :#色块码垛
                        run_app_status=2
                        color_Palletizer.init()
                        beep()
                    elif string.find("#FaceTrack!") >= 0 :#人脸识别
                        run_app_status=3
                        face_Track.init()
                        beep()
                    elif string.find("#PTZColorTrace!") >= 0 :#颜色追踪
                        run_app_status=4
                        color_Trace.init()
                        beep()
                    elif string.find("#ApriltagSort!") >= 0 :#二维码标签分拣
                        run_app_status=5
                        apriltag_Sort.init()
                        beep()
                    elif string.find("#ApriltagStack!") >= 0 :#二维码标签码垛
                        run_app_status=6
                        apriltag_Palletizer.init()
                        beep()
                    elif string.find("#ApriltagNumSort!") >= 0 :#二维码标签数字分拣
                        run_app_status=7
                        apriltag_Num_Sort.init()
                        beep()
                    elif string.find("#NumTrack!") >= 0 :#数字追踪
                        run_app_status=8
                        num_track.init()
                        beep()
                    elif string.find("#ApriltagTrack!") >= 0 :#二维码追踪
                        run_app_status=9
                        april_tag_track.init()
                        beep()
            except Exception as e:#串口数据异常进入
                print('Error:', e)

        if run_app_status==1:
            color_Sort.run(cx,cy,cz)#运行分拣色块功能
        elif run_app_status==2:
            color_Palletizer.run(cx,cy,cz)#运行色块码垛功能
        elif run_app_status==3:
            face_Track.run()#运行人脸追踪
        elif run_app_status==4:
            color_Trace.run()#运行颜色追踪
        elif run_app_status==5:
            apriltag_Sort.run(cx,cy,cz)#运行标签分拣
        elif run_app_status==6:
            apriltag_Palletizer.run(cx,cy,cz)#运行标签码垛
        elif run_app_status==7:
            apriltag_Num_Sort.run(cx,cy,cz)#数字分拣
        elif run_app_status==8:
            num_track.run()#数字识别追踪
        elif run_app_status==9:
            april_tag_track.run()#二维码追踪




