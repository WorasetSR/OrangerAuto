import cv2
import numpy as np

def generate_marker():
    # ใช้ Dictionary 4x4_50 ตามที่กำหนดไว้ใน vision_tracker.py
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    # สร้างรูป ArUco ID = 0, ขนาด 400x400 พิกเซล
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, 0, 400)
    
    # บันทึกเป็นไฟล์รูปภาพ
    filename = "aruco_marker_id0.png"
    cv2.imwrite(filename, marker_img)
    print(f"สร้างไฟล์ {filename} สำเร็จแล้ว!")
    print("นำไฟล์นี้ไปปริ้นลงกระดาษ A4 ได้เลยครับ")

if __name__ == "__main__":
    generate_marker()
