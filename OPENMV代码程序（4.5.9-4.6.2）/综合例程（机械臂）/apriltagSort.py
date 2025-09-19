# =========================  头文件  =========================
import sensor, time, math
from pyb import Pin, Timer, UART

# =========================  主类  =========================
class ApriltagSort():
    # --------------  串口 & LED --------------
    uart = UART(3, 115200)
    uart.init(115200, bits=8, parity=None, stop=1)

    tim = Timer(4, freq=1000)                # 1 kHz PWM
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
    led_dac.pulse_width_percent(50)          # 默认 50% 亮度

    # --------------  关键状态变量 --------------
    target_tag_id = 0       # 当前要处理的 AprilTag ID
    move_x = 150            # 机械臂 X 坐标（mm）
    move_y = 0              # 机械臂 Y 坐标（mm）
    mid_block_cx = 80       # 图像中心 X（QQVGA 160×120）
    mid_block_cy = 60       # 图像中心 Y
    mid_block_cnt = 0       # 连续对准计数，用于防抖
    move_status = 0         # 0找抓取点 1抓取中 2找放置框 3放置中
    block_degress = 0       # 抓取目标的旋转角（°）

    # 如需“抓取A→放置B”的映射，可在此配置
    # 例：place_id_map = {1:10, 2:20}
    place_id_map = {}

    # =========================  初始化摄像头  =========================
    def init(self):
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QQVGA)  # 160×120
        sensor.skip_frames(n=2000)
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)

        self.led_dac.pulse_width_percent(0)   # 关灯
        
        # 先让机械臂回到初始位置
        self.uart.write("$KMS:{:03d},{:03d},{:03d},1000!\n"
                        .format(int(self.move_x), int(self.move_y), 70))
        time.sleep_ms(1000)

    # =========================  选最近 Tag  =========================
    def _pick_best_tag(self, tags):
        if not tags:
            return None
        cx, cy = self.mid_block_cx, self.mid_block_cy
        # 用距离平方选离中心最近的 tag
        best = min(tags, key=lambda t: (t.cx - cx) ** 2 + (t.cy - cy) ** 2)
        return best

    # =========================  主循环  =========================
    def run(self, cx=0, cy=0, cz=0):
        """
        cx,cy,cz：微调抓取/放置时的偏移量，单位 mm
        """
        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy
        have_target = False
        img = sensor.snapshot()

        place_id = self.place_id_map.get(self.target_tag_id, self.target_tag_id)

        # --------------  阶段0/1：找抓取目标  --------------
        tags = img.find_apriltags()
        if self.move_status < 2:
            tag = self._pick_best_tag(tags)
            if tag:
                # 画框、画十字、写 ID
                img.draw_rectangle(tag.rect, color=(255, 0, 0))
                img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))
                img.draw_string(tag.x, tag.y - 10, "{}".format(tag.id), color=(255, 0, 0))
                # 保存目标信息
                block_cx, block_cy = tag.cx, tag.cy
                self.block_degress = 180 * tag.rotation / math.pi
                self.target_tag_id = tag.id
                have_target = True

        # --------------  阶段2/3：找放置框  --------------
        else:
            # 支持“抓取ID→放置ID”映射
            place_id = self.place_id_map.get(self.target_tag_id, self.target_tag_id)
            # 过滤出目标放置框
            tags = [t for t in tags if t.id == place_id] or tags
            tag = self._pick_best_tag(tags)
            if tag:
                img.draw_rectangle(tag.rect, color=(255, 255, 255))
                img.draw_cross(tag.cx, tag.cy, size=2, color=(255, 255, 255))
                img.draw_string(tag.x, tag.y - 10, "dst:{}".format(tag.id), color=(255, 255, 255))
                block_cx, block_cy = tag.cx, tag.cy
                have_target = True

        # =========================  状态机 =========================
        if have_target or (self.move_status == 1):  # 有目标或已抓取
            # ---- 阶段0：对准抓取点 ----
            if self.move_status == 0:
                if abs(block_cx - self.mid_block_cx) > 3:
                    self.move_y += 0.2 if block_cx > self.mid_block_cx else -0.2
                if abs(block_cy - self.mid_block_cy) > 3:
                    self.move_x += 0.3 if block_cy < self.mid_block_cy or self.move_x <= 80 else -0.3
                # 连续对准 10 次后进入抓取
                if abs(block_cx - self.mid_block_cx) <= 3 and abs(block_cy - self.mid_block_cy) <= 3:
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 10:
                        self.mid_block_cnt = 0
                        self.move_status = 1
                else:
                    self.mid_block_cnt = 0
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                    .format(int(self.move_x), -int(self.move_y), 70, 10))
                time.sleep_ms(10)

            # ---- 阶段1：抓取动作 ----
            elif self.move_status == 1:
                self.move_status = 2
                # 计算夹爪旋转 PWM
                deg = self.block_degress % 90
                spin_calw = int(1500 - deg * 500 / 90) if deg < 45 \
                            else int((90 - deg) * 500 / 90 + 1500)
                spin_calw = max(500, min(2500, spin_calw))

                # 发送旋转、张开
                self.uart.write("{{#004P{:04d}T1000!}}".format(spin_calw))
                # 根据偏移量重新计算坐标
                l = math.sqrt(self.move_y ** 2 + self.move_x ** 2)
                sin, cos = self.move_x / l, self.move_y / l
                self.move_y = (l + 85 + cy) * cos + cx
                self.move_x = (l + 85 + cy) * sin
                time.sleep_ms(100)
                self.uart.write("{#005P1300T1000!}")        # 张开爪子
                time.sleep_ms(100)

                # 移动到目标上方
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x) - 30, -int(self.move_y) + 10, 120, 1000))
                time.sleep_ms(1200)
                # 下降
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x) - 30 - 10, -int(self.move_y) + 10, 5 + cz, 1000))
                time.sleep_ms(1200)
                self.uart.write("{#005P1750T1000!}")        # 闭合爪子
                time.sleep_ms(1200)
                # 抬起
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y), 120, 1000))
                time.sleep_ms(1200)
                self.uart.write("{#004P1500T1000!}")        # 爪子回正
                time.sleep_ms(100)

                # 过渡到放置区上方
                if place_id==1:
                    self.move_y=-120
                    self.move_x=-50
                elif place_id==2:
                    self.move_y=-120
                    self.move_x=30
                elif place_id==3:
                    self.move_y=-120
                    self.move_x=80

                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y), 120, 1000))
                time.sleep_ms(1200)
                self.mid_block_cnt = 0

            # ---- 阶段2：对准放置框（方向与阶段0相反） ----
            elif self.move_status == 2:
                if abs(block_cx - self.mid_block_cx) > 3:
                    self.move_x += 0.3 if block_cx > self.mid_block_cx else -0.3
                if abs(block_cy - self.mid_block_cy) > 3:
                    self.move_y += 0.2 if block_cy > self.mid_block_cy else -0.2
                # 连续对准 5 次后进入放置
                if abs(block_cx - self.mid_block_cx) <= 3 and abs(block_cy - self.mid_block_cy) <= 3:
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 5:
                        self.mid_block_cnt = 0
                        self.move_status = 3
                else:
                    self.mid_block_cnt = 0
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                    .format(int(self.move_x), -int(self.move_y), 120, 10))
                time.sleep_ms(10)

            # ---- 阶段3：放下并归位 ----
            elif self.move_status == 3:
                self.move_status = 0
                l = math.sqrt(self.move_y ** 2 + self.move_x ** 2)
                sin, cos = self.move_x / l, self.move_y / l
                self.move_y = (l + 85 + cy) * cos
                self.move_x = (l + 85 + cy) * sin
                # 移动到放置点上方
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y) - 35, 120, 1000))
                time.sleep_ms(1200)
                if place_id == 1: # 1
                    # 下降
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                    .format(int(self.move_x) + 10, -int(self.move_y) - 35 - 10, 5 + cz, 1000))

                elif place_id == 2:
                    # 下降
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                    .format(int(self.move_x) - 15 -10, -int(self.move_y) - 35, 5 + cz, 1000))

                elif place_id == 3:
                    # 下降
                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                    .format(int(self.move_x) - 20 - 20, -int(self.move_y) - 25, 5 + cz, 1000))

                time.sleep_ms(1200)
                self.uart.write("{#005P1100T1000!}")        # 张开爪子放下
                time.sleep_ms(1200)

                # 抬起
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y) - 35, 120, 1000))
                time.sleep_ms(1200)
                # 复位
                self.move_y, self.move_x = 0, 150
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y), 70, 1000))
                time.sleep_ms(1200)
                self.mid_block_cnt = 0
                self.target_tag_id = 0      # 清掉目标，等待下一次


# =========================  程序入口  =========================
if __name__ == "__main__":
    app = ApriltagSort()
    app.init()
    while True:
        app.run(0, 0, 0)
