import sensor, image, time, ml, math, uos, gc
from pyb import UART

class GarbageSorting:
    # ---------- 模型 ----------
    net   = None
    labels= None
    try:
        net = ml.Model("garbage.tflite",
                       load_to_fb=uos.stat('garbage.tflite')[6] > (gc.mem_free() - 64*1024))
    except Exception as e:
        raise Exception('Failed to load "garbage.tflite". ' + str(e))

    try:
        labels = [l.rstrip('\n') for l in open("labels.txt")]
    except Exception as e:
        raise Exception('Failed to load "labels.txt". ' + str(e))

    min_confidence = 0.5
    colors = [(255,0,0),(0,255,0),(255,255,0),(0,0,255),
              (255,0,255),(0,255,255),(255,255,255)]

    # ---------- 外设 ----------
    uart = UART(3, 115200)
    uart.init(115200, bits=8, parity=None, stop=1)

    # ---------- 状态 ----------
    stage = 0   # 0: 正方形检测阶段, 1: 模型识别阶段
    center_cnt = 0  # 中心检测计数器
    target_rect = None  # 保存识别到的正方形

    # 图像中心
    IMG_W = 160
    IMG_H = 120
    MID_X = IMG_W // 2
    MID_Y = IMG_H // 2

    # ---------- 初始化 ----------
    def init(self):
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QQVGA)  # 160x120
        sensor.skip_frames(n=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        time.sleep_ms(1000)

    # ---------- 阶段1：找正方形 ----------
    def detect_square(self, img):
        img.lens_corr(1.8)   # 畸变矫正

        rects = []
        for th in [20000, 10000, 5000]:  # 自适应阈值
            rects = img.find_rects(threshold=th)
            if rects:
                break

        for r in rects:
            img.draw_rectangle(r.rect(), color=(255,255,255))
            cx = r.x() + r.w()//2
            cy = r.y() + r.h()//2
            img.draw_cross(cx, cy, color=(255,0,0))

            # 判断是否接近画面中心 (±20像素)
            if abs(cx - self.MID_X) < 20 and abs(cy - self.MID_Y) < 20:
                self.center_cnt += 1
                if self.center_cnt >= 10:   # 连续10帧有效
                    self.uart.write("RECT_OK\n")
                    self.stage = 1
                    self.target_rect = r  # 保存ROI
                    self.center_cnt = 0
                    return True
            else:
                self.center_cnt = 0
        return False

    # ---------- 阶段2：模型识别 ----------
    def classify_object(self, img):
        if not self.target_rect:
            return

        # ROI 扩展 (避免裁剪太紧)
        x, y, w, h = self.target_rect.rect()
        expand = 5
        x = max(0, x - expand)
        y = max(0, y - expand)
        w = min(self.IMG_W - x, w + 2 * expand)
        h = min(self.IMG_H - y, h + 2 * expand)
        roi_img = img.copy(roi=(x, y, w, h))   # 关键字参数

        roi = (x, y, w, h)
        roi_img = img.copy(roi=(x, y, w, h))

        logits = self.net.predict([roi_img])[0].flatten().tolist()
        cls    = max(range(len(logits)), key=lambda i: logits[i])
        score  = logits[cls]

        if score >= self.min_confidence:
            img.draw_rectangle(roi, color=(0,255,0))
            img.draw_string(x, y-10, "{}:{:.2f}".format(self.labels[cls], score),
                            color=self.colors[cls % len(self.colors)], scale=2)
            self.uart.write("CLS:{},{}\n".format(self.labels[cls], score))

    # ---------- 主循环 ----------
    def run(self):
        img = sensor.snapshot()

        if self.stage == 0:
            self.detect_square(img)
        elif self.stage == 1:
            self.classify_object(img)


if __name__ == "__main__":
    app = GarbageSorting()
    app.init()
    while True:
        app.run()
