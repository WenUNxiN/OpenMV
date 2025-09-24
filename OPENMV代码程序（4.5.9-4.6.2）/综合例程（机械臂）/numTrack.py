# 数字追踪（单目标 PID 云台版）——精简注释版
import sensor, image, time, math, uos, gc, ml
from pyb import Pin, Timer, UART

class NumTrack:
    # ---------------- 类静态变量 ----------------
    min_confidence = 0.7
    colors = [(255,0,0),(0,255,0),(255,255,0),(0,0,255),(255,0,255),(0,255,255),(255,255,255)]
    threshold_list = [(int(min_confidence * 255), 255)]

    # 加载模型与标签（仅一次）
    try:
        net = ml.Model("trained.tflite",
                       load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - 64*1024))
        labels = [l.rstrip('\n') for l in open("labels.txt")]
    except Exception as e:
        raise Exception("模型/标签加载失败: " + str(e))

    # 硬件资源
    uart = UART(3, 115200)
    uart.init(115200, bits=8, parity=None, stop=1)
    tim = Timer(4, freq=1000)
    led = tim.channel(1, Timer.PWM, pin=Pin("P7"))
    led.pulse_width_percent(50)          # 默认亮度

    # 图像中心参考
    mid_cx, mid_cy = 80, 60

    # 舵机初始 PWM（μs）
    servo0, servo1 = 1500, 1250

    # ---------------- 初始化 ----------------
    def init(self, cx=80.5, cy=60.5):
        """只调图像中心与舵机起始位置，不再重复配置摄像头"""
        self.mid_cx, self.mid_cy = cx, cy
        self.led.pulse_width_percent(10)   # 降低照明
        # 四舵机一次性复位（0/1 云台，2/3 备用）
        self.uart.write("{{#000P{:04d}T1100!#001P{:04d}T1100!#002P1750T1100!#003P0860T1100!}}\n"
                        .format(self.servo0, self.servo1))
        time.sleep_ms(1000)

    # ---------------- TFLite 后处理 ----------------
    def fomo_post_process(self, model, inputs, outputs):
        """把 heatmap 转成框列表，每类一个 list"""
        _, oh, ow, oc = model.output_shape[0]
        roi = inputs[0].roi
        scale = min(roi[2]/ow, roi[3]/oh)
        x_offset = (roi[2] - ow*scale)/2 + roi[0]
        y_offset = (roi[3] - oh*scale)/2 + roi[1]

        res = [[] for _ in range(oc)]
        for i in range(oc):
            hm = image.Image(outputs[0][0, :, :, i] * 255)
            for b in hm.find_blobs(self.threshold_list, x_stride=1, y_stride=1,
                                   area_threshold=1, pixels_threshold=1):
                x, y, w, h = b.rect()
                score = hm.get_statistics(thresholds=self.threshold_list, roi=b.rect()).l_mean() / 255.0
                # 映射回原图
                x = int(x*scale + x_offset)
                y = int(y*scale + y_offset)
                w = int(w*scale)
                h = int(h*scale)
                res[i].append((x, y, w, h, score))
        return res

    # ---------------- 主循环 ----------------
    def run(self):
        """只追踪 self.cap_num_status 对应的数字"""
        img = sensor.snapshot()
        block_cx, block_cy = self.mid_cx, self.mid_cy   # 默认值

        # 1. 推理并画框
        for i, dets in enumerate(self.net.predict([img], callback=self.fomo_post_process)):
            if i == 0 or not dets: continue
            for x, y, w, h, score in dets:
                if i == 3:           # 这里固定追类别 3，可改 self.cap_num_status
                    block_cx, block_cy = x, y
                    img.draw_rectangle(x, y, w, h, color=(255, 255, 255))
                    img.draw_cross(block_cx, block_cy, size=2, color=(255, 0, 0))
                    img.draw_string(x, y-10, self.labels[i], color=(255, 255, 255))

        # 2. 简单 P 控制云台
        dead_x, dead_y = 8, 4
        if abs(block_cx - 80) > dead_x:
            self.servo0 += int((80 - block_cx) * 0.4)
        if abs(block_cy - 60) > dead_y:
            self.servo1 += int((block_cy - 60) * 0.35)

        # 限幅
        self.servo0 = max(650, min(2400, self.servo0))
        self.servo1 = max(500, min(2400, self.servo1))

        # 3. 立即发送（T0000 表示最快速度）
        self.uart.write("{{#000P{:04d}T0000!#001P{:04d}T0000!}}\n".format(self.servo0, self.servo1))
        time.sleep_ms(50)   # 控制周期 20 Hz


# ---------------- 上电运行 ----------------
if __name__ == "__main__":
    app = NumTrack()
    app.init()     # 设置中心与初始舵机角
    while True:
        app.run()  # 持续追踪