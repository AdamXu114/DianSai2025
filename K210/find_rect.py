import sensor, image, time
import lcd
from machine import UART
from fpioa_manager import fm

# 串口初始化
fm.register(6, fm.fpioa.UART2_RX)
fm.register(8, fm.fpioa.UART2_TX)
uart = UART(UART.UART2, 115200, 8, 1, 0, timeout=1000, read_buf_len=4096)

# LCD 初始化
lcd.init()
lcd.clear(lcd.RED)                  # Clear lcd screen.

# 图像初始化
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

# 稳定性判断参数
STABLE_FRAMES = 5         # 连续帧数
THRESHOLD = 2             # 坐标变化阈值（像素）
stable_count = 0          # 当前稳定帧计数
last_corners = None       # 上一帧的顶点坐标
sent = False              # 是否已发送过

while True:
    clock.tick()
    img = sensor.snapshot()
    rects = img.find_rects(threshold=10000)

    if rects:
        # 只处理第一个检测到的矩形
        corners = rects[0].corners()  # [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]

        if last_corners:
            # 计算所有顶点与上一帧的最大差值
            max_diff = 0
            for (x,y), (lx,ly) in zip(corners, last_corners):
                diff = abs(x-lx) + abs(y-ly)
                if diff > max_diff:
                    max_diff = diff

            if max_diff <= THRESHOLD:
                stable_count += 1
            else:
                stable_count = 0
                sent = False  # 波动后允许重新发送
        else:
            stable_count = 1  # 第一帧先计数

        last_corners = corners

        # 当达到连续稳定帧并且未发送时，执行一次发送
        if stable_count >= STABLE_FRAMES and not sent:
            x1, y1 = corners[0]
            x2, y2 = corners[1]
            x3, y3 = corners[2]
            x4, y4 = corners[3]
            uart.write("Corners: ({},{}) ({},{}) ({},{}) ({},{})\r\n"
                       .format(x1, y1, x2, y2, x3, y3, x4, y4))
            sent = True

        # 可视化
        img.draw_rectangle(rects[0].rect(), color=(255, 0, 0))
        for p in corners:
            img.draw_circle(p[0], p[1], 5, color=(0, 255, 0))
    else:
        # 没检测到矩形时，重置状态
        stable_count = 0
        last_corners = None
        sent = False
        uart.write("No rectangle detected.\r\n")

    lcd.display(img)
    # （可选）打印 FPS
    # print("FPS:", clock.fps())
