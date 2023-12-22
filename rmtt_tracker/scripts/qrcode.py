#!/usr/bin/env python3
# coding=utf-8
# 环境准备：
# sudo apt-get install libzbar-dev
# pip3 install pyzbar

# 导入所需的库
import cv2
import pyzbar.pyzbar as pyzbar

# 定义一个函数，用于解码并显示二维码
def decode_display(image):
    # 使用pyzbar库解码图像中的二维码
    barcodes = pyzbar.decode(image)
    for barcode in barcodes:
        # 提取二维码的边界框位置
        # 绘制二维码的边界框
        (x, y, w, h) = barcode.rect
        cv2.rectangle(image, (x, y), (x + w, y + h), (225, 225, 225), 2)

        # 提取二维码数据为字节对象，需要将其转换为字符串
        barcode_data = barcode.data.decode("utf-8")
        barcode_type = barcode.type

        # 在图像上绘制二维码数据和类型
        text = "{} ({})".format(barcode_data, barcode_type)
        cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 225, 225), 2)

        # 打印二维码数据和类型到终端
        print("[INFO] x: {} y: {} w: {} h: {} find {} code: {}".format(x, y, w, h, barcode_type, barcode_data))
    return image

# 定义检测函数
def detect():
    # 打开摄像头
    camera = cv2.VideoCapture(0)

    while True:
        # 读取当前帧
        ret, frame = camera.read()
        # 转为灰度图像
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        im = decode_display(gray)

        cv2.waitKey(5)
        cv2.imshow("QR code detection", im)
        # 如果按键q则跳出循环
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    detect()

