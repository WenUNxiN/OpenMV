import sensor, time, math
from pyb import Pin, Timer, UART

class ColorShapeTrace():
    # ========== 颜色阈值 ==========
    red_threshold    = (0, 100,  20, 127,   0, 127)
    blue_threshold   = (0, 100, -128, 127, -128, -15)
    green_threshold  = (0, 100, -128, -28,   0,  70)
    yellow_threshold = (57, 100, -33,  70,  48, 127)

    # ========== 运行时可切换 ==========
    track_color_threshold = yellow_threshold   # 当前追踪颜色
    track_shape           = "circle"           # "circle" 或 "rect"

    # ========== 串口 & PWM ==========
    uart = UART(3, 115200)
    uart.init(115200, bits=8, parity=None, stop=1)

    tim = Timer(4, freq=1000)
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=0)

    # ========== 默认值 ==========
    mid_block_cx = 80
    mid_block_cy = 60
    servo0 = 1500
    servo1 = 1500

    # ------------------------------------------------------------
    def init(self):
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QQVGA)
        sensor.skip_frames(n=2000)
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)

        self.led_dac.pulse_width_percent(100)

        # 初始机械臂位置
        self.servo0 = 1500
        self.servo1 = 1500
        self.uart.write("{{#000P{:0>4d}T1100!#001P{:0>4d}T1100!#002P{:0>4d}T1100!#003P{:0>4d}T1100!}}\n"
                        .format(self.servo0, self.servo1, 1800, 860))
        time.sleep_ms(1000)

    # ------------------------------------------------------------
    def run(self):
        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy

        img = sensor.snapshot()
        blobs = img.find_blobs([self.track_color_threshold],
                               x_stride=15, y_stride=15,
                               pixels_threshold=25)

        target_blob = None
        max_score = 0

        for b in blobs:
            # 1. 面积越大分越高
            area_score = b.w() * b.h()

            # 2. 圆度计算
            perimeter = b.perimeter()
            roundness = (4 * math.pi * b.pixels()) / (perimeter * perimeter) if perimeter else 0

            # 3. 根据追踪模式打分
            if self.track_shape == "circle":
                score = area_score * roundness        # 圆形优先
            else:
                score = area_score                    # 普通矩形

            if score > max_score:
                max_score = score
                target_blob = b

        # 如果找到了目标
        if target_blob:
            x, y, w, h = target_blob.rect()
            cx, cy = target_blob.cx(), target_blob.cy()

            # 画框、画十字
            img.draw_rectangle(x, y, w, h, color=(255, 255, 255))
            img.draw_cross(cx, cy, size=2, color=(255, 0, 0))
            label = "C" if self.track_shape == "circle" else "R"
            img.draw_string(x, y-10, label, color=(255, 0, 0))

            block_cx, block_cy = cx, cy

        # ---------------- 舵机控制 ----------------
        if abs(block_cx - 80) >= 5:
            move_x = -0.8 * abs(block_cx - 80) if block_cx > 80 else 0.8 * abs(block_cx - 80)
            self.servo0 = int(self.servo0 + move_x)

        if abs(block_cy - 60) >= 2:
            move_y = -1 * abs(block_cy - 60) if block_cy > 60 else 1 * abs(block_cy - 60)
            self.servo1 = int(self.servo1 + move_y)

        # 限幅
        self.servo0 = max(min(self.servo0, 2400), 650)
        self.servo1 = max(min(self.servo1, 2400), 500)

        self.uart.write("{{#000P{:0>4d}T0000!#001P{:0>4d}T0000!}}\n"
                        .format(self.servo0, self.servo1))
        time.sleep_ms(50)