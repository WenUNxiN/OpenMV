# ========================== 1. 库导入 ==========================
import sensor, time, math            # sensor：OpenMV 相机驱动；time：延时；math：数学计算
from pyb import Pin, Timer, UART     # pyb：OpenMV 的 PyBoard 硬件抽象层（GPIO、定时器、串口）
# ========================== 2. 主类定义 ==========================
class ColorSort():
    """
    功能：
    1. 用颜色识别（红/绿/蓝）找到目标物块；
    2. 通过串口指挥 STM32 机械臂：
         ‑ 走到物块上方 → 下降 → 抓取 → 抬升 → 旋转到对应颜色放置区 → 放下 → 归位；
    3. 支持 3 个阶段的状态机：
         0：寻找并对准物块
         1：抓取动作
         2：寻找放置区并对准
         3：放下并复位
    """

    # ---------------- 类变量：颜色阈值（LAB 空间） ----------------
    red_threshold    = (0, 100,  20, 127,   0, 127)
    blue_threshold   = (0, 100, -128, 127,-128, -15)
    green_threshold  = (0, 49, -22, -39, 74, 2)

    # ---------------- 硬件资源：串口 + LED PWM ----------------
    uart = UART(3, 115200)
    uart.init(115200, bits=8, parity=None, stop=1)

    tim = Timer(4, freq=1000)                    # 1 kHz PWM
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"))
    led_dac.pulse_width_percent(100)             # 开机背光 100%

    # ---------------- 状态机 / 运动控制变量 ----------------
    cap_color_status = 0      # 当前要抓的颜色：0 未选，'R'/'G'/'B'
    move_y = 0                # 机械臂 Y（视觉坐标系转过来的）
    move_x = 150              # 机械臂 X（初始待命位）
    mid_block_cx = 80         # 画面中心
    mid_block_cy = 60
    mid_block_cnt = 0         # 连续对准计数器
    move_status = 0           # 状态机阶段

    def clamp(self):
        self.uart.write("{#005P1600T1000!}")#机械爪抓取物块

    def loosen(self):
        self.uart.write("{#005P1200T1000!}")#机械爪松开物块
        
    # ============================================================
    # 函数：init() —— 摄像头、串口、机械臂的初始化
    # ============================================================
    def init(self):
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)   # 彩色
        sensor.set_framesize(sensor.QQVGA)    # 160×120
        sensor.skip_frames(n=2000)            # 等待稳定
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)

        # 状态变量清零
        self.cap_color_status = 0
        self.move_y = 0
        self.move_x = 150
        self.mid_block_cnt = 0
        self.move_status = 0

        self.led_dac.pulse_width_percent(100)
        
        # 机械臂回到待命位
        self.uart.write("$KMS:{:03d},{:03d},{:03d},1000!\n"
                        .format(int(self.move_x), int(self.move_y), 120))
        time.sleep_ms(1000)

    # ============================================================
    # 函数：run(cx, cy, cz)
    # 参数：手动微调偏移量（调试用）
    #   cx：前后微调（+ 向后，- 向前）
    #   cy：左右微调（+ 向右，- 向左）
    #   cz：高度微调（+ 更高，- 更低）
    # ============================================================
    def run(self, cx=0, cy=0, cz=0):
        block_cx = self.mid_block_cx        # 默认目标中心为画面中心
        block_cy = self.mid_block_cy
        color_read_succed = 0               # 是否识别到色块
        color_status = 0                    # 本轮识别到的颜色字符

        # ---------------- 1. 图像采集 ----------------
        img = sensor.snapshot()

        # ---------------- 2. 颜色识别 ----------------
        # 只检测红 / 蓝 / 绿
        red_blobs   = img.find_blobs([self.red_threshold],   x_stride=15, y_stride=15, pixels_threshold=25)
        blue_blobs  = img.find_blobs([self.blue_threshold],  x_stride=15, y_stride=15, pixels_threshold=25)
        green_blobs = img.find_blobs([self.green_threshold], x_stride=15, y_stride=15, pixels_threshold=50)

        # 2.1 红色优先
        if red_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'R'):
            color_status = 'R'
            color_read_succed = 1
            for y in red_blobs:
                img.draw_rectangle(y.rect(), color=(255, 255, 255))
                img.draw_cross(y.cx(), y.cy(), size=2, color=(255, 0, 0))
                img.draw_string(y.x(), y.y()-10, "red", color=(255, 0, 0))
                block_cx = y.cx()
                block_cy = y.cy()

        # 2.2 蓝色其次
        elif blue_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'B'):
            color_status = 'B'
            color_read_succed = 1
            for y in blue_blobs:
                img.draw_rectangle(y.rect(), color=(255, 255, 255))
                img.draw_cross(y.cx(), y.cy(), size=2, color=(0, 0, 255))
                img.draw_string(y.x(), y.y()-10, "blue", color=(0, 0, 255))
                block_cx = y.cx()
                block_cy = y.cy()

        # 2.3 绿色最后
        elif green_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'G'):
            color_status = 'G'
            color_read_succed = 1
            for y in green_blobs:
                img.draw_rectangle(y.rect(), color=(255, 255, 255))
                img.draw_cross(y.cx(), y.cy(), size=2, color=(0, 255, 0))
                img.draw_string(y.x(), y.y()-10, "green", color=(0, 255, 0))
                block_cx = y.cx()
                block_cy = y.cy()

        # ---------------- 3. 状态机 ----------------
        if color_read_succed or (self.move_status == 1):
            # ----------- 阶段 0：对准目标色块 -----------
            if self.move_status == 0:
                if abs(block_cx - self.mid_block_cx) > 3:
                    if block_cx > self.mid_block_cx:
                        self.move_y += 0.2     # 右 -> 机械臂 Y+
                    else:
                        self.move_y -= 0.2
                if abs(block_cy - self.mid_block_cy) > 3:
                    if block_cy > self.mid_block_cy and self.move_x > 80:
                        self.move_x -= 0.3     # 前 -> 机械臂 X-
                    else:
                        self.move_x += 0.3

                # 连续 10 次对准才算 OK
                if abs(block_cy - self.mid_block_cy) <= 3 and abs(block_cx - self.mid_block_cx) <= 3:
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 10:
                        self.mid_block_cnt = 0
                        self.move_status = 1           # 进入抓取阶段
                        self.cap_color_status = color_status  # 锁定颜色
                else:
                    self.mid_block_cnt = 0

                # 实时发坐标
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y), 120, 10))
                time.sleep_ms(100)

            # ----------- 阶段 1：抓取动作 -----------
            elif self.move_status == 1:
                self.move_status = 2
                time.sleep_ms(100)

                # 1. 张开爪子
                self.loosen()
                time.sleep_ms(1000)

                # 2. 计算机械臂末端到目标直线下落点（含微调）
                l = math.sqrt(self.move_y**2 + self.move_x**2)
                sin = self.move_x / l
                cos = self.move_y / l
                self.move_y = (l + 85 + cy) * cos + cx   # 85 = 经验补偿
                self.move_x = (l + 85 + cy) * sin

                # 3. 先移到物块上方
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x) - 25, -int(self.move_y) + 10, 120, 1000))
                time.sleep_ms(1000)

                # 4. 下降
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x) - 25 - 10, -int(self.move_y) + 10, 5 + cz, 1000))
                time.sleep_ms(1200)

                # 5. 合爪
                self.clamp()
                time.sleep_ms(1200)

                # 6. 抬升
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y), 120, 1000))
                time.sleep_ms(1200)

                # 6. 抬升
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(150, 0, 120, 1000))
                time.sleep_ms(1200)

                # 7. 旋转到对应颜色的放置区
                self.move_y = 120
                self.move_x = 30
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y), 120, 1000))
                time.sleep_ms(1200)

                self.mid_block_cnt = 0

            # ----------- 阶段 2：对准放置区 -----------
            elif self.move_status == 2:
                if abs(block_cx - self.mid_block_cx) > 3:
                    if block_cx > self.mid_block_cx and self.move_x > -200:
                        self.move_x -= 0.3
                    else:
                        self.move_x += 0.3
                if abs(block_cy - self.mid_block_cy) > 3:
                    if block_cy > self.mid_block_cy:
                        self.move_y -= 0.2
                    else:
                        self.move_y += 0.2

                if abs(block_cy - self.mid_block_cy) <= 3 and abs(block_cx - self.mid_block_cx) <= 3:
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 5:
                        self.mid_block_cnt = 0
                        self.move_status = 3
                        self.cap_color_status = color_status
                else:
                    self.mid_block_cnt = 0

                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y), 120, 10))
                time.sleep_ms(10)

            # ----------- 阶段 3：放下并复位 -----------
            elif self.move_status == 3:
                self.move_status = 0
                # 计算放置点
                l = math.sqrt(self.move_y**2 + self.move_x**2)
                sin = self.move_x / l
                cos = self.move_y / l
                self.move_y = (l + 85 + cy) * cos
                self.move_x = (l + 85 + cy) * sin

                # 移动到放置区上方
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x) - 10, -int(self.move_y) + 30, 120, 1000))
                time.sleep_ms(1000)

                if color_status == 'R':
                    # 下降
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                    .format(int(self.move_x) + 5, -int(self.move_y) + 30, 5 + cz + 100, 1000))
                    time.sleep_ms(1200)
                if color_status == 'G':
                    # 下降
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                    .format(int(self.move_x) - 10, -int(self.move_y) + 30, 5 + cz + 100, 1000))
                    time.sleep_ms(1200)
                if color_status == 'B':
                    # 下降
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                    .format(int(self.move_x) - 10, -int(self.move_y) + 30, 5 + cz + 100, 1000))
                    time.sleep_ms(1200)

                # 张开爪子放物块
                self.loosen()
                time.sleep_ms(1200)

                # 抬升
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y), 120, 1000))
                time.sleep_ms(1200)

                # 回到待命位
                self.move_y = 0
                self.move_x = 150
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y), 120, 1000))
                time.sleep_ms(1200)

                # 状态复位
                self.mid_block_cnt = 0
                self.cap_color_status = 0

# ========================== 4. 主程序入口 ==========================
if __name__ == "__main__":
    colorSort = ColorSort()
    colorSort.init()
    while True:
        colorSort.run(0, 0, 0)   # 持续循环
