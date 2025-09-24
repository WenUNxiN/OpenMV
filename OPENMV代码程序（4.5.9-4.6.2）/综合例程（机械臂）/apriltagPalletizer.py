# 二维码码垛
# ========================== 1. 库导入 ==========================
import sensor, time, math     # sensor：OpenMV 摄像头库；time：延时；math：数学计算
from pyb import Pin, Timer, UART  # pyb：OpenMV 的 PyBoard 硬件抽象层（GPIO、定时器、串口）

# ========================== 2. 主类定义 ==========================
class ApriltagPalletizer():
    """
    用 AprilTag 做视觉定位，然后通过串口指挥 STM32 机械臂完成：
    1. 找到 AprilTag
    2. 移动到标签正上方
    3. 抓取标签所在物块
    4. 把物块码垛到固定位置
    5. 重复 3 次
    """

    # ---------------- 硬件资源：串口 + PWM 控制 LED ----------------
    uart = UART(3, 115200)           # 串口 3，波特率 115200，与 STM32 统一
    uart.init(115200, bits=8, parity=None, stop=1)

    tim = Timer(4, freq=1000)        # 定时器4，1kHz，用于 LED 背光 PWM
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
    led_dac.pulse_width_percent(50)  # 默认 50% 亮度

    # ---------------- 状态机/运动控制变量 ----------------
    cap_color_status = 0  # 保留变量，未使用
    move_y = 0            # 机械臂 Y 轴坐标（视觉坐标系转换后的）
    move_x = 150          # 机械臂 X 轴坐标（初始位置）
    mid_block_cx = 80     # 画面中心 x（视觉中心）
    mid_block_cy = 60     # 画面中心 y
    mid_block_cnt = 0     # 连续对准计数器，>10 次视为真正对准，避免抖动
    move_status = 0       # 0：寻找 AprilTag；1：执行抓取流程
    block_degress = 0     # AprilTag 的旋转角（°）
    palletizer_cnt = 0    # 已完成的码垛次数（最多 3 次）

    def clamp(self):
        self.uart.write("{#005P1600T1000!}")#机械爪抓取物块

    def loosen(self):
        self.uart.write("{#005P1200T1000!}")#机械爪松开物块
    # ============================================================
    # 函数：init()
    # 功能：初始化摄像头、串口、LED、机械臂归位
    # ============================================================
    def init(self):
        sensor.reset()                               # 复位摄像头
        sensor.set_pixformat(sensor.RGB565)          # 彩色
        sensor.set_framesize(sensor.QQVGA)           # 160×120
        sensor.skip_frames(n=2000)                   # 等待稳定
        sensor.set_auto_gain(True)                   # 自动增益开
        sensor.set_auto_whitebal(True)               # 自动白平衡开

        # 再次初始化串口（冗余，保险）
        self.uart.init(115200, bits=8, parity=None, stop=1)
        self.led_dac.pulse_width_percent(0)          # 关闭 LED

        # 状态变量复位
        self.cap_color_status = 0
        self.move_y = 0
        self.move_x = 150
        self.mid_block_cnt = 0
        self.move_status = 0
        self.palletizer_cnt = 0
        self.block_degress = 0

        # 机械臂回到“待命位”
        # 协议：$KMS:x,y,z,speed!
        self.uart.write("$KMS:{:03d},{:03d},{:03d},1000!\n"
                        .format(int(self.move_x), int(self.move_y), 70))
        time.sleep_ms(1000)

    # ============================================================
    # 函数：run(cx,cy,cz)
    # 参数：手动微调偏移量（调试阶段用）
    #   cx：左右偏移  cy：前后偏移  cz：高度偏移
    # ============================================================
    def run(self, cx=0, cy=0, cz=0):
        block_cx = self.mid_block_cx   # 先假设中心就是视野中心
        block_cy = self.mid_block_cy
        color_read_succed = 0          # 是否检测到 AprilTag

        # -------------------- 1. 拍一张图 --------------------
        img = sensor.snapshot()

        # -------------------- 2. 找 AprilTag ----------------
        for tag in img.find_apriltags():        # 默认 TAG36H11
            img.draw_rectangle(tag.rect, color=(255, 0, 0))  # 画红框
            img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))# 画绿十字
            img.draw_string(tag.x, tag.y-10,
                            "{}".format(tag.id), color=(255, 0, 0))
            block_cx = tag.cx
            block_cy = tag.cy
            # tag.rotation 为弧度，转成角度方便后面爪子旋转对准
            self.block_degress = 180 * tag.rotation / math.pi
            color_read_succed = 1
            break  # 找到第一个即可

        # -------------------- 3. 状态机 ---------------------
        # 条件：只要看到 AprilTag 或者已经在执行抓取流程就继续
        if color_read_succed == 1 or self.move_status == 1:
            # ------------ 阶段 0：对准 AprilTag ------------
            if self.move_status == 0:
                # --- 3.1 根据视觉误差调整 move_x / move_y ---
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

                # --- 3.2 连续 10 次都对准才算完成 ---
                if abs(block_cy - self.mid_block_cy) <= 3 and \
                   abs(block_cx - self.mid_block_cx) <= 3:
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 10:
                        self.mid_block_cnt = 0
                        self.move_status = 1   # 进入抓取阶段
                else:
                    self.mid_block_cnt = 0     # 误差过大，清零

                # --- 3.3 实时发送坐标给 STM32 ---
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x),
                                        -(int(self.move_y)),  # 注意负号！
                                        70, 10))
                time.sleep_ms(10)

            # ------------ 阶段 1：执行抓取 ------------
            elif self.move_status == 1:
                self.move_status = 0   # 立即清 0，防止重复进入

                # --- 4.1 计算爪子旋转角度 ---
                # 把 tag 的旋转映射到舵机脉宽 500~2500，中位 1500
                if self.block_degress % 90 < 45:
                    spin_calw = int(1500 - self.block_degress % 90 * 500 / 90)
                else:
                    spin_calw = int((90 - self.block_degress % 90) * 500 / 90 + 1500)
                if spin_calw >= 2500 or spin_calw <= 500:
                    spin_calw = 1500   # 越界保护

                # --- 4.2 旋转爪到合适角度并张开 ---
                self.uart.write("{{#004P{:0^4}T1000!}}".format(spin_calw))  # 旋转
                time.sleep_ms(100)
                self.loosen()  # 张开爪子
                time.sleep_ms(1000)

                # --- 4.3 计算下降深度（考虑微调量 cy/cz）---
                l = math.sqrt(self.move_y**2 + self.move_x**2)
                sin = self.move_x / l
                cos = self.move_y / l
                self.move_y = (l + 85 + cy) * cos + cx      # 85 = 爪子到物块距离经验值
                self.move_x = (l + 85 + cy) * sin

                # --- 4.4 先移动到物块正上方 ---
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x) - 30,
                                        -(int(self.move_y)) + 10,
                                        70, 1000))
                time.sleep_ms(100)

                # --- 4.5 再下降到物块 ---
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x) - 30 - 10,
                                        -(int(self.move_y)) + 10,
                                        5 + cz, 1000))
                time.sleep_ms(1200)

                # --- 4.6 合爪抓取 ---
                self.clamp()
                time.sleep_ms(1200)

                # --- 4.7 抬升 ---
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x),
                                        -(int(self.move_y)),
                                        120, 1000))
                time.sleep_ms(1200)

                # --- 4.8 旋转到码垛区上方 ---
                self.move_y = -190
                self.move_x = 20
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x),
                                        -(int(self.move_y)),
                                        120, 1000))
                time.sleep_ms(1200)

                # --- 4.9 根据码垛次数决定放置高度 ---
                if self.palletizer_cnt == 0:
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                    .format(int(self.move_x),
                                            -(int(self.move_y)),
                                            10 + cz, 1000))
                elif self.palletizer_cnt == 1:
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                    .format(int(self.move_x),
                                            -(int(self.move_y)),
                                            10 + cz + 30, 1000))
                elif self.palletizer_cnt == 2:
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                    .format(int(self.move_x),
                                            -(int(self.move_y)),
                                            10 + cz + 30 + 30, 1000))
                time.sleep_ms(1200)

                # --- 4.10 放爪、抬升、回待命位 ---
                self.loosen()   # 张开爪子放物块
                time.sleep_ms(1200)
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x),
                                        -(int(self.move_y)),
                                        120, 1000))
                time.sleep_ms(1200)

                # 机械臂回到初始待命位
                self.move_y = 0
                self.move_x = 150
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x),
                                        -(int(self.move_y)),
                                        70, 1000))
                time.sleep_ms(1200)

                # 完成一次循环
                self.move_status = 0
                self.palletizer_cnt += 1   # 下一次码垛

# ========================== 5. 主程序入口 ==========================
if __name__ == "__main__":
    app = ApriltagPalletizer()  # 创建对象
    app.init()                  # 初始化摄像头、机械臂
    while True:
        app.run(0, 0, 0)        # 死循环，实时运行
