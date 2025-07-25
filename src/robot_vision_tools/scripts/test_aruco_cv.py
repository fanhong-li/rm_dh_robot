#!/usr/bin/env python3

import cv2
import numpy as np

print(f"OpenCV版本: {cv2.__version__}")

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
try:
    detector = cv2.aruco.ArucoDetector(aruco_dict)
    use_new_api = True
except AttributeError:
    try:
        parameters = cv2.aruco.DetectorParameters_create()
        use_new_api = False
    except AttributeError:
        parameters = cv2.aruco.DetectorParameters()
        use_new_api = False

img = cv2.imread('test_image.png')

if img is None:
    print("图片未找到，请确认 test_image.png 存在")
    exit(1)

print(f"图片尺寸: {img.shape}")

if use_new_api:
    corners, ids, rejected = detector.detectMarkers(img)
else:
    corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=parameters)

if ids is not None:
    print(f"检测到的marker ID: {ids.flatten()}")
    print(f"检测到 {len(ids)} 个marker")
    img_marked = cv2.aruco.drawDetectedMarkers(img.copy(), corners, ids)
    cv2.imwrite('detected_markers.png', img_marked)
    print("检测结果已保存为 detected_markers.png")
    try:
        cv2.imshow("Detected Aruco", img_marked)
        print("按任意键关闭窗口...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except:
        print("无法显示窗口，但结果已保存")
else:
    print("没有检测到任何Aruco marker")
    print("可能原因:")
    print("1. marker不在画面中或被遮挡")
    print("2. marker打印不清晰或损坏")
    print("3. 光线反射或过暗")
    print("4. marker类型不匹配（当前使用DICT_4X4_50）")
    cv2.imwrite('original_image.png', img)
    print("原图已保存为 original_image.png，可以手动检查marker是否清晰可见") 