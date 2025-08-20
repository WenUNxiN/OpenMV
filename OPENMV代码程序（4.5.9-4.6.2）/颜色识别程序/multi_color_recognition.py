# 引入必需的库
import sensor   #导入相机模块
import image    #导入图像处理模块
import time     #导入时间模块

# ---------- 1. 摄像头初始化 ----------
sensor.reset()                     # 复位摄像头，确保每次上电都是干净状态
sensor.set_pixformat(sensor.RGB565) # 设置像素格式为 RGB565（彩色，每个像素 16 bit）
sensor.set_framesize(sensor.QVGA)   # 分辨率 320×240，速度与清晰度折中
sensor.skip_frames(time=1000)       # 跳过前 1 s 的帧，让自动曝光/白平衡稳定
sensor.set_auto_gain(False)         # 关闭自动增益，防止颜色随亮度变化而漂移
sensor.set_auto_whitebal(False)     # 关闭自动白平衡，颜色阈值才稳定

# ---------- 2. 颜色阈值（LAB 色彩空间） ----------
# LAB 顺序：(L_min, L_max, A_min, A_max, B_min, B_max)
# 数值来源于 Tools → Threshold Editor，实际使用时可根据光照再微调
RED   = (25, 85,  20, 127,  15, 127)   # 红色阈值
GREEN = (30,100, -80, -20, -10,  50)   # 绿色阈值
BLUE  = (20, 90, -10,  60,-128, -30)   # 蓝色阈值
YELLOW= (30,100, -30,  30,  30, 127)   # 黄色阈值

# colors 列表：把“名字-阈值-显示颜色”打包在一起，后面循环用
colors = [
    ("RED",    RED,    (255,  0,  0)),  # 红框 + 红字
    ("GREEN",  GREEN,  (  0,255,  0)),  # 绿框 + 绿字
    ("BLUE",   BLUE,   (  0,  0,255)),  # 蓝框 + 蓝字
    ("YELLOW", YELLOW, (255,255,  0))   # 黄框 + 黄字
]

# ---------- 3. 主循环 ----------
clock = time.clock()  # 创建时钟对象，用来计算 FPS
while True:
    clock.tick()               # 更新 FPS 计时
    img = sensor.snapshot()    # 抓取一帧图像到 img

    # 对 4 种颜色依次做检测
    for name, th, rgb in colors:
        # 在当前图像里查找符合颜色阈值 th 的色块
        # pixels_threshold 和 area_threshold 过滤掉过小的噪点
        for blob in img.find_blobs([th],
                                   pixels_threshold=200,  # 色块像素 ≥ 200
                                   area_threshold=200,    # 色块面积 ≥ 200
                                   merge=True):           # 合并相邻色块
            # 画矩形框：左上角 (blob.x(), blob.y())，宽高 (blob.w(), blob.h())
            img.draw_rectangle(blob.rect(), color=rgb, thickness=2)
            # 在矩形框上方 15 像素处写颜色名字，颜色与框一致，字号 2
            img.draw_string(blob.x(),
                            blob.y() - 15,
                            name,
                            color=rgb,
                            scale=2)

    # 如果想在串口终端看 FPS，可取消下一行注释：
    # print("FPS:", clock.fps())
