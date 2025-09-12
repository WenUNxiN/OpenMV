import sensor, image, time, math
from pyb import Pin, Timer, UART

class FaceTrack():
    # ---------- 类变量 ----------
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.HQVGA)      # 240×160
    sensor.skip_frames(n=2000)
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)

    face_cascade = image.HaarCascade("frontalface", stages=10)
    print(face_cascade)

    uart = UART(3, 115200)
    uart.init(115200, bits=8, parity=None, stop=1)

    tim = Timer(4, freq=1000)
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
    led_dac.pulse_width_percent(50)

    mid_block_cx = 120      # 画面中心
    mid_block_cy = 80

    servo0 = 1500           # 水平舵机
    servo1 = 1500           # 垂直舵机

    # ---------- 初始化 ----------
    def init(self, cx=120, cy=80):
        sensor.reset()
        sensor.set_pixformat(sensor.GRAYSCALE)
        sensor.set_framesize(sensor.HQVGA)
        sensor.skip_frames(n=2000)
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)

        self.uart.init(115200, bits=8, parity=None, stop=1)
        self.led_dac.pulse_width_percent(0)

        self.mid_block_cx = cx
        self.mid_block_cy = cy

        self.servo0 = 1500
        self.servo1 = 1250

        # 四个舵机回中
        self.uart.write("{{#000P{:0>4d}T1500!#001P{:0>4d}T1500!}}\n"
                        .format(self.servo0, self.servo1))

    # ---------- 主循环 ----------
    def run(self):
        img = sensor.snapshot()
        faces = img.find_features(self.face_cascade, threshold=0.75, scale=1.2)

        if faces:
            # 最大人脸
            max_blob = max(faces, key=lambda b: b[2] * b[3])
            face = max_blob
            cx = int(face[0] + face[2] / 2)
            cy = int(face[1] + face[3] / 2)

            img.draw_rectangle(face, color=(255, 255, 255))
            img.draw_cross(cx, cy)

            # ====== 方案 A：比例 + 增量限幅（简单有效） ======
            dead_x, dead_y = 6, 4          # 死区再放大一点
            kp_x, kp_y = 0.12, -0.15       # kp_y 取负值，消除方向相反的问题
            max_step = 4                   # 每帧最大步进再减小

            err_x = (self.mid_block_cx - cx) * kp_x
            err_y = (self.mid_block_cy - cy) * kp_y   # 负号已放到 kp_y 里了

            move_x = max(-max_step, min(max_step, int(err_x)))
            move_y = max(-max_step, min(max_step, int(err_y)))
            # ---------------------------------------------------------

            # 如需方案 B EMA，请把上面两行换成下面两行：
            # move_x = self.ema_filter(err_x, 0.25, 'x')
            # move_y = self.ema_filter(err_y, 0.25, 'y')

            # 如需方案 C PID，请把上面两行换成：
            # move_x = self.pid(err_x, 'x')
            # move_y = self.pid(err_y, 'y')

            if self.servo0>2400: self.servo0=2400
            elif self.servo0<650: self.servo0=650
            if self.servo1>2400: self.servo1=2400
            elif self.servo1<500: self.servo1=500

            # 更新舵机
            self.servo0 = max(650, min(2400, self.servo0 + move_x))
            self.servo1 = max(500, min(2400, self.servo1 + move_y))

            self.uart.write("{{#000P{:0>4d}T0000!#001P{:0>4d}T0000!}}\n"
                            .format(self.servo0, self.servo1))
            time.sleep_ms(10)

    # ---------- 方案 B：EMA 滤波 ----------
    # alpha: 0~1，越小越平滑
    _ema_x, _ema_y = 0.0, 0.0

    def ema_filter(self, err, alpha, axis):
        if axis == 'x':
            self._ema_x = alpha * err + (1 - alpha) * self._ema_x
            return int(self._ema_x)
        else:
            self._ema_y = alpha * err + (1 - alpha) * self._ema_y
            return int(self._ema_y)

    # ---------- 方案 C：PID ----------
    # 仅做 PI，Kd=0
    _intg_x, _intg_y = 0.0, 0.0
    _last_x, _last_y = 0.0, 0.0
    Kp_x, Ki_x = 0.25, 0.03
    Kp_y, Ki_y = 0.30, 0.04

    def pid(self, err, axis):
        if axis == 'x':
            self._intg_x += err
            self._intg_x = max(-100, min(100, self._intg_x))
            out = int(self.Kp_x * err + self.Ki_x * self._intg_x)
            return max(-10, min(10, out))     # 增量限幅
        else:
            self._intg_y += err
            self._intg_y = max(-100, min(100, self._intg_y))
            out = int(self.Kp_y * err + self.Ki_y * self._intg_y)
            return max(-10, min(10, out))

# ---------- 入口 ----------
if __name__ == "__main__":
    app = FaceTrack()
    app.init()
    while True:
        app.run()






