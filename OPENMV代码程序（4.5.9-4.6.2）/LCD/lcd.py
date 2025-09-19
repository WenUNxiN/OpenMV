#在不连接PC时，也可以通过LCD查看帧缓存区的画面。
# 导入 sensor（摄像头）和 display（显示屏）模块
import sensor, display

# 复位并初始化摄像头
sensor.reset()

# 设置像素格式为 RGB565（彩色）
sensor.set_pixformat(sensor.RGB565)

# 设置帧大小为 QQVGA2（128×160），与显示屏分辨率一致
sensor.set_framesize(sensor.QQVGA2)

# 初始化 SPI 接口的显示屏（默认使用 GPIO 控制背光）
lcd = display.SPIDisplay()

# 无限循环，持续更新画面
while True
    # 抓取一帧图像并立即显示到 LCD 屏幕上
    lcd.write(sensor.snapshot())