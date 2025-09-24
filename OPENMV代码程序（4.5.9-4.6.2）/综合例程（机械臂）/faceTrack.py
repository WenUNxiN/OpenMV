# 人脸追踪
import sensor, image, time, math
from pyb import Pin, Timer, UART

class FaceTrack:
    # -------------------- 类变量（仅执行一次） --------------------
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)  # 灰度图，节省算力
    sensor.set_framesize(sensor.HQVGA)      # 240×160
    sensor.skip_frames(n=2000)              # 等待画质稳定
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)

    face_cascade = image.HaarCascade("frontalface", stages=10)  # 加载人脸级联
    print(face_cascade)                      # 打印级联信息，调试用

    uart = UART(3, 115200)                   # 与舵控板通信
    uart.init(115200, bits=8, parity=None, stop=1)

    tim = Timer(4, freq=1000)                # LED 照明 PWM
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"))
    led_dac.pulse_width_percent(50)          # 默认 50% 亮度

    mid_cx, mid_cy = 120, 80   # 画面中心坐标（HQVGA）
    servo0, servo1 = 1500, 1250  # 水平/垂直舵机初始脉宽（us）

    # -------------------- 实例初始化（可重复调用） --------------------
    def init(self, cx=120, cy=80):
        # sensor 已在类变量初始化，此处不再 reset
        self.uart.init(115200, bits=8, parity=None, stop=1)
        self.led_dac.pulse_width_percent(0)  # 关闭照明

        self.mid_cx, self.mid_cy = cx, cy
        self.servo0, self.servo1 = 1500, 1250

        # 四轴全部回中（仅 0/1 号参与人脸跟踪）
        self.uart.write("{{#000P{:04d}T1100!#001P{:04d}T1100!"
                        "#002P1750T1100!#003P0860T1100!}}\n"
                        .format(self.servo0, self.servo1))

    # -------------------- 主循环 --------------------
    def run(self):
        img = sensor.snapshot()
        faces = img.find_features(self.face_cascade,
                                  threshold=0.75, scale=1.2)

        if not faces:        # 无人脸直接结束本次循环
            return

        # 取最大人脸
        face = max(faces, key=lambda b: b[2]*b[3])
        cx = face[0] + face[2]//2
        cy = face[1] + face[3]//2
        img.draw_rectangle(face, color=255)
        img.draw_cross(cx, cy)

        # --------------- 控制算法（三选一，默认比例） ---------------
        dead_x, dead_y = 6, 4      # 死区
        kp_x, kp_y   = 0.12, -0.15  # 方向修正：kp_y 为负
        max_step     = 4            # 单帧最大步进

        err_x = (self.mid_cx - cx) * kp_x
        err_y = (self.mid_cy - cy) * kp_y

        move_x = max(-max_step, min(max_step, int(err_x)))
        move_y = max(-max_step, min(max_step, int(err_y)))

        # 如需 EMA 平滑，把上面两行换成：
        # move_x = self.ema_filter(err_x, 0.25, 'x')
        # move_y = self.ema_filter(err_y, 0.25, 'y')

        # 如需 PID，把上面两行换成：
        # move_x = self.pid(err_x, 'x')
        # move_y = self.pid(err_y, 'y')

        # --------------- 更新舵机 ---------------
        self.servo0 = max(650,  min(2400, self.servo0 + move_x))
        self.servo1 = max(500,  min(2400, self.servo1 + move_y))

        self.uart.write("{{#000P{:04d}T0000!#001P{:04d}T0000!}}\n"
                        .format(self.servo0, self.servo1))
        time.sleep_ms(10)   # 给舵控板一点执行时间

    # -------------------- EMA 平滑 --------------------
    _ema_x, _ema_y = 0.0, 0.0

    def ema_filter(self, err, alpha, axis):
        if axis == 'x':
            self._ema_x = alpha * err + (1-alpha) * self._ema_x
            return int(self._ema_x)
        else:
            self._ema_y = alpha * err + (1-alpha) * self._ema_y
            return int(self._ema_y)

    # -------------------- PI 控制器 --------------------
    _intg_x, _intg_y = 0.0, 0.0
    Kp_x, Ki_x = 0.25, 0.03
    Kp_y, Ki_y = 0.30, 0.04

    def pid(self, err, axis):
        if axis == 'x':
            self._intg_x += err
            self._intg_x = max(-100, min(100, self._intg_x))
            out = int(self.Kp_x * err + self.Ki_x * self._intg_x)
            return max(-10, min(10, out))
        else:
            self._intg_y += err
            self._intg_y = max(-100, min(100, self._intg_y))
            out = int(self.Kp_y * err + self.Ki_y * self._intg_y)
            return max(-10, min(10, out))

# -------------------- 程序入口 --------------------
if __name__ == "__main__":
    app = FaceTrack()
    app.init()
    while True:
        app.run()