import sensor, image, lcd, time

# 初始化摄像头和LCD显示屏
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # 颜色跟踪必须关闭自动增益
sensor.set_auto_whitebal(False)  # 颜色跟踪必须关闭白平衡
clock = time.clock()
lcd.init()  # 初始化LCD显示

# 定义用于红色和绿色识别的LAB阈值
# LAB格式：(L_min, L_max, A_min, A_max, B_min, B_max)

red_threshold =   (43, 100, 12, 117, -47, 92)   # 红色阈值
green_threshold = (35, 100, -96, 25, 10, 127)   # 绿色阈值

thresholds = [red_threshold, green_threshold]  # 放入列表中

color_labels = ["Red", "Green"]  # 用于标识颜色

while True:
    clock.tick()
    img = sensor.snapshot()  # 获取摄像头图像

    # 查找所有符合阈值的颜色块
    blobs = img.find_blobs(thresholds, pixels_threshold=10, merge=True)

    if blobs:
        for i, blob in enumerate(blobs):
            x, y, w, h = blob.rect()
            center_x, center_y = blob.cx(), blob.cy()

            # 根据索引来判断是红还是绿
            color_index = blob.code() - 1  # code() 返回1或2对应第一个/第二个阈值
            color_name = color_labels[color_index]

            # 绘制矩形框和中心点
            img.draw_rectangle((x, y, w, h), color=(255, 0, 0) if color_name == "Red" else (0, 255, 0))
            img.draw_cross(center_x, center_y)



    # 显示在LCD上
    lcd.display(img)

    # 打印帧率
    print("FPS:", clock.fps())
