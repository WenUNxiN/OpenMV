# OpenMV 黄色正方形识别
import sensor, image, time, math, pyb

# 黄色 LAB 阈值（用 Tools → Threshold Editor 微调）
# colour_threshold = (55, 90, -25, 10, 30, 80)
# 蓝色
colour_threshold = (85, 11, 35, -122, -124, -9)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160×120
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()

# 边长差异容忍度（0~1）
EDGE_RATIO_TOL = 0.15

while True:
    clock.tick()
    img = sensor.snapshot()

    # 1. 先找黄色色块
    blobs = img.find_blobs(
        [colour_threshold],
        pixels_threshold=100,   # 面积太小直接过滤
        area_threshold=100,
        merge=True)

    for b in blobs:
        # 2. 用最小外接矩形近似正方形
        w = b.w()
        h = b.h()
        if w == 0 or h == 0:
            continue

        # 边长差异
        if abs(w - h) / max(w, h) > EDGE_RATIO_TOL:
            continue

        # 3. 画出结果
        cx = b.cx()
        cy = b.cy()
        img.draw_rectangle(b.rect(), color=(255, 255, 0), thickness=2)
        img.draw_cross(cx, cy, size=5, color=(255, 255, 0))

        # 4. 计算旋转角（以水平向右为 0°）
        corners = b.min_corners()        # 返回 4 个角点
        dx = corners[1][0] - corners[0][0]
        dy = corners[1][1] - corners[0][1]
        angle = math.degrees(math.atan2(dy, dx))

        # 5. 串口输出
        print("Square x=%d y=%d angle=%.1f" % (cx, cy, angle))

    print("FPS:", clock.fps())
