import sensor, image, time

# 红色阈值（LAB颜色空间）
# 用 OpenMV IDE → Tools → Machine Vision → Threshold Editor 调整
red_threshold = (0, 100, 30, 127, 20, 127)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160x120，速度快
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)         # 关自动增益
sensor.set_auto_whitebal(False)     # 关自动白平衡
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    # 找红色区域（找色块）
    blobs = img.find_blobs(
        [red_threshold],
        pixels_threshold=50,      # 最小像素数（过滤小噪声）
        area_threshold=50,        # 最小区域面积
        merge=True                # 合并重叠色块
    )

    if blobs:
        # 取面积最大的红色区域
        biggest = max(blobs, key=lambda b: b.pixels())

        # 计算圆度（2*sqrt(pi*area)/周长 接近1说明是圆）
        roundness = (2 * (3.14159 * biggest.pixels()) ** 0.5) / biggest.perimeter()
        if roundness > 0.8:  # 圆度阈值，可调
            cx = biggest.cx()
            cy = biggest.cy()
            r = int((biggest.w() + biggest.h()) / 4)  # 粗略估算半径
            img.draw_circle(cx, cy, r, color=(255, 0, 0), thickness=2)
            print("Circle at x=%d y=%d r=%d Roundness=%.2f" % (cx, cy, r, roundness))
        else:
            print("Red blob detected but not round")
    else:
        print("No red blob found")

    print("FPS:", clock.fps())
