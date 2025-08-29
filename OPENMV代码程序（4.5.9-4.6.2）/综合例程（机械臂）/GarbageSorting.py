# =========================  头文件  =========================
"""
Garbage-Sorting Arm (OpenMV H7)  -  FOMO + 正方形框选 版本
1. 先跑 FOMO 检测 → 取最高置信目标 → 画正方形 ROI
2. 视觉伺服对准 → 抓取 → 搬运 → 颜色桶精对准 → 放下
"""
import sensor, image, time, ml, uos, gc, math
from pyb import Pin, Timer, UART

# =========================  主类  =========================
class GarbageSortingArm:
    # ----------------------------------------------------------
    # 1. 模型加载（FOMO）
    # ----------------------------------------------------------
    net   = None
    labels= None
    min_confidence = 0.3     # 可以适当调低便于调试

    try:
        net = ml.Model("garbage.tflite",
                       load_to_fb=uos.stat('garbage.tflite')[6] > (gc.mem_free() - 64*1024))
    except Exception as e:
        raise Exception('Failed to load "garbage.tflite". ' + str(e))

    try:
        labels = [l.rstrip('\n') for l in open("garbage.txt")]
    except Exception as e:
        raise Exception('Failed to load "garbage.txt". ' + str(e))

    threshold_list = [(int(min_confidence*255), 255)]

    # 官方 FOMO 后处理（全局全图检测）
    @staticmethod
    def fomo_post(model, inputs, outputs):
        ob, oh, ow, oc = model.output_shape[0]
        x_scale = inputs[0].roi[2] / ow
        y_scale = inputs[0].roi[3] / oh
        scale   = min(x_scale, y_scale)
        x_off   = ((inputs[0].roi[2] - ow*scale)/2) + inputs[0].roi[0]
        y_off   = ((inputs[0].roi[3] - oh*scale)/2) + inputs[0].roi[1]

        results = [[] for _ in range(oc)]
        for c in range(oc):
            heat = image.Image(outputs[0][0, :, :, c]*255)
            for b in heat.find_blobs(GarbageSortingArm.threshold_list,
                                     x_stride=1, y_stride=1,
                                     area_threshold=1, pixels_threshold=1):
                x,y,w,h = b.rect()
                score = heat.get_statistics(thresholds=GarbageSortingArm.threshold_list,
                                            roi=b.rect()).l_mean()/255.0
                x = int(x*scale + x_off)
                y = int(y*scale + y_off)
                w = int(w*scale)
                h = int(h*scale)
                results[c].append((x, y, w, h, score))
        return results

    colors = [(255,0,0),(0,255,0),(255,255,0),(0,0,255),
              (255,0,255),(0,255,255),(255,255,255)]

    # ----------------------------------------------------------
    # 2. 串口 & LED
    # ----------------------------------------------------------
    uart = UART(3, 115200)
    uart.init(115200, bits=8, parity=None, stop=1)
    tim = Timer(4, freq=1000)
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=0)

    # ----------------------------------------------------------
    # 3. 业务常数
    # ----------------------------------------------------------
    place_id_map = {"harmful": 1, "kitchen": 2, "recoverable": 3}

    coarse_pos = {
        "harmful"   : (-20,  120),
        "kitchen"   : (20, 150),
        "recoverable": (50,120)
    }#放置区

    color_th = {
        "red"  : [(0, 100,  20, 127,   0, 127)],
        "green": [(0, 49, -4, -44, 98, -68)],
        "blue" : [(0, 100, -128, 127,-128, -15)]
    }#放置区颜色

    color_id = {"red": 1, "green": 2, "blue": 3}

    # ----------------------------------------------------------
    # 4. 状态机
    # ----------------------------------------------------------
    stage        = 0      # 0 检测+对准  1 抓取  2 放置区对准  3 放下
    target_rect  = None   # 正方形 ROI： (x,y,w,h)
    target_class = None
    last_cls     = None
    cls_cnt      = 0

    move_x = 150
    move_y = 0
    mid_block_cnt = 0
    move_status   = 0

    IMG_W = 160
    IMG_H = 120
    MID_X = IMG_W // 2
    MID_Y = IMG_H // 2

    # ==========================================================
    # 初始化
    # ==========================================================
    def init(self):
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QQVGA)
        sensor.skip_frames(n=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        time.sleep_ms(1000)

        self.led_dac.pulse_width_percent(10)
        self.uart.write("$KMS:{:03d},{:03d},{:03d},1000!\n"
                        .format(int(self.move_x), int(self.move_y), 50))
        time.sleep_ms(1000)

    # ==========================================================
    # 工具函数
    # ==========================================================
    def detect_and_square(self, img):
           """
           先按优先级挑类别，再取该类里置信最高的目标
           """
           detections = self.net.predict([img], callback=self.fomo_post)

           # 优先级表，越靠前越高
           priority = ["harmful", "kitchen", "recoverable"]

           chosen = None          # (x,y,w,h,label,score)
           for lbl in priority:
               cls_id = self.labels.index(lbl) if lbl in self.labels else -1
               if cls_id < 0:
                   continue
               det_list = detections[cls_id]
               if det_list:
                   # 取该类别里置信最高的
                   best = max(det_list, key=lambda d: d[4])
                   x, y, w, h, score = best
                   chosen = (x, y, w, h, lbl, score)
                   break   # 只要优先级最高的类别里有目标就结束

           if chosen is None:
               self.target_rect = None
               return None

           # 画正方形 ROI
           cx = chosen[0] + chosen[2]//2
           cy = chosen[1] + chosen[3]//2
           side = max(chosen[2], chosen[3]) + 10
           x = max(0, cx - side//2)
           y = max(0, cy - side//2)
           side = min(side, self.IMG_W - x, self.IMG_H - y)
           self.target_rect = (x, y, side, side)

           img.draw_rectangle((x, y, side, side), color=(0, 255, 0))
           img.draw_string(x, y-10, "{}:{:.2f}".format(chosen[4], chosen[5]),
                           color=self.colors[self.labels.index(chosen[4]) % len(self.colors)], scale=2)
           return chosen[4], chosen[5]

    def find_color_blob(self, img, color_name):
        blobs = img.find_blobs(self.color_th[color_name], pixels_threshold=150)
        if not blobs:
            return None
        blob = max(blobs, key=lambda b: b.pixels())
        img.draw_rectangle(blob.rect(), color=(255, 255, 255))
        img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 255))
        return blob

    # ==========================================================
    # 主循环
    # ==========================================================
    def run(self, cx=0, cy=0, cz=0):
        img = sensor.snapshot()

        # ---------- stage 0：检测 + 正方形框选 + 对准 ----------
        if self.stage == 0:
            label_score = self.detect_and_square(img)
            if label_score:
                label, score = label_score

                if label in self.place_id_map:
                    x, y, w, h = self.target_rect
                    cx_sq = x + w//2
                    cy_sq = y + h//2
                    # 微小步对准
                    if abs(cx_sq - self.MID_X) > 3:
                        self.move_y += 0.3 if cx_sq > self.MID_X else -0.3
                    if abs(cy_sq - self.MID_Y) > 3:
                        self.move_x += 0.3 if cy_sq < self.MID_Y else -0.3

                    self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                    .format(int(self.move_x), -int(self.move_y), 50, 10))
                    time.sleep_ms(100)

                    # 连续 10 帧稳定
                    if abs(cx_sq - self.MID_X) <= 5 and abs(cy_sq - self.MID_Y) <= 5:
                        if label == self.last_cls:
                            self.cls_cnt += 1
                        else:
                            self.cls_cnt = 1
                            self.last_cls = label
                        if self.cls_cnt >= 10:
                            self.target_class = label
                            self.stage = 1
                            self.move_status = 0
                    else:
                        self.cls_cnt = 0
                        self.last_cls = None
                else:
                    self.cls_cnt = 0
                    self.last_cls = None
            else:
                self.cls_cnt = 0
                self.last_cls = None

        # ---------- stage 1：抓取 ----------
        if self.stage == 1:
            if self.move_status == 0:
                # 1. 张开
                self.uart.write("{#005P1200T1000!}")
                time.sleep_ms(1000)

                # 2. 计算落点（补偿）
                l = math.sqrt(self.move_y**2 + self.move_x**2)
                sin = self.move_x / l
                cos = self.move_y / l
                self.move_y = (l + 85 + cy) * cos
                self.move_x = (l + 85 + cy) * sin

                # 3. 移动到上方
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x)-25, -int(self.move_y)+10, 120, 1000))
                time.sleep_ms(1000)

                # 4. 下降
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x)-25, -int(self.move_y)+10, 5+cz, 1000))
                time.sleep_ms(1200)

                # 5. 合爪
                self.uart.write("{#005P1650T1000!}")
                time.sleep_ms(1200)

                # 6. 抬起
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y), 120, 1000))
                time.sleep_ms(1200)

                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(150, 0, 120, 1000))
                time.sleep_ms(1200)

                # 7. 前往放置区粗位置
                # 根据垃圾类型选粗位置
                x0, y0 = self.coarse_pos[self.target_class]
                self.move_x = x0
                self.move_y = y0
                self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                                .format(int(self.move_x), -int(self.move_y), 120, 1000))

                self.stage = 2
                self.move_status = 0

        # ---------- stage 2：放置区精对准 ----------
        if self.stage == 2:
            if self.move_status == 0:
                self.uart.write("$KMS:{:03d},{:03d},{:03d},1000!\n"
                                .format(0, -120, 120))
                time.sleep_ms(1200)
                self.move_status = 2
                self.mid_block_cnt = 0
                return

            color_name = {"harmful":"red", "kitchen":"green", "recoverable":"blue"}[self.target_class]
            blob = self.find_color_blob(img, color_name)
            if not blob:
                self.mid_block_cnt = 0
                return
            block_cx, block_cy = blob.cx(), blob.cy()

            if abs(block_cx - self.MID_X) > 3:
                self.move_x += 0.3 if block_cx < self.MID_X else -0.3
            if abs(block_cy - self.MID_Y) > 3:
                self.move_y += 0.2 if block_cy < self.MID_Y else -0.2

            if abs(block_cy - self.MID_Y) <= 3 and abs(block_cx - self.MID_X) <= 3:
                self.mid_block_cnt += 1
                if self.mid_block_cnt > 5:
                    self.mid_block_cnt = 0
                    self.stage = 3
            else:
                self.mid_block_cnt = 0

            self.uart.write("$KMS:{:03d},{:03d},{:03d},10!\n"
                            .format(int(self.move_x), -int(self.move_y), 120))
            time.sleep_ms(10)

        # ---------- stage 3：放下 & 复位 ----------
        if self.stage == 3:
            l = math.sqrt(self.move_y**2 + self.move_x**2)
            sin = self.move_x / l
            cos = self.move_y / l
            self.move_y = (l + 85 + cy) * cos
            self.move_x = (l + 85 + cy) * sin

            self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                            .format(int(self.move_x), -int(self.move_y), 120, 1000))
            time.sleep_ms(1000)

            self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                            .format(int(self.move_x), -int(self.move_y), 5+cz+90, 1000))
            time.sleep_ms(1200)

            self.uart.write("{#005P1200T1000!}")   # 张开
            time.sleep_ms(1200)

            self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                            .format(int(self.move_x), -int(self.move_y), 120, 1000))
            time.sleep_ms(1200)

            self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                            .format(0, -120, 120, 1000))
            time.sleep_ms(1200)

            self.move_y = 0
            self.move_x = 150
            self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                            .format(int(self.move_x), -int(self.move_y), 120, 1000))
            time.sleep_ms(1200)

            self.uart.write("$KMS:{:03d},{:03d},{:03d},{:03d}!\n"
                            .format(int(self.move_x), -int(self.move_y), 50, 1000))
            time.sleep_ms(1200)

            self.stage = 0
            self.cls_cnt = 0
            self.last_cls = None
            self.target_class = None
            self.mid_block_cnt = 0

# =========================  程序入口  =========================
if __name__ == "__main__":
    app = GarbageSortingArm()
    app.init()
    while True:
        app.run(0, 0, 0)
