import cv2
import numpy as np
import json
import os

CONFIG_FILE = "hsv_config.json"

# Default HSV values for 6 colors
hsv_ranges = {
    'red':    {'lower': [0, 100, 100], 'upper': [10, 255, 255]},
    'orange': {'lower': [11, 100, 100], 'upper': [25, 255, 255]},
    'yellow': {'lower': [26, 100, 100], 'upper': [35, 255, 255]}, # Used for some tuning if needed
    'green':  {'lower': [36, 100, 100], 'upper': [85, 255, 255]},
    'cyan':   {'lower': [86, 100, 100], 'upper': [105, 255, 255]},
    'blue':   {'lower': [106, 100, 100], 'upper': [135, 255, 255]},
    'purple': {'lower': [136, 100, 100], 'upper': [170, 255, 255]}
}

def load_config():
    global hsv_ranges
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
            hsv_ranges = json.load(f)
            print(f"Loaded config from {CONFIG_FILE}")

def save_config():
    with open(CONFIG_FILE, 'w') as f:
        json.dump(hsv_ranges, f, indent=4)
    print(f"Saved config to {CONFIG_FILE}")

def nothing(x):
    pass


# เก็บเฟรมปัจจุบันไว้ใน mutable container เพื่อให้ mouse callback (ซึ่งถูกเรียก
# แบบ async นอก loop หลัก) เข้าถึงเฟรมล่าสุดได้เสมอ
_latest_frame = {"frame": None}

# ระยะขอบเขตที่ auto-set รอบๆ ค่าที่คลิก (ปรับได้ถ้าอยากได้ช่วงกว้าง/แคบกว่านี้)
H_MARGIN = 10
S_MARGIN = 60
V_MARGIN = 60


def on_mouse_click(event, x, y, flags, param):
    """
    คลิกซ้ายที่จุดใดก็ได้บนหน้าต่าง 'Original' เพื่อ "จิ้มสี" ของหินจริงตรงนั้น
    แล้วให้โปรแกรม auto-set trackbar ทั้ง 6 ตัวให้เป็นช่วงรอบๆ ค่าที่จิ้มได้เลย
    แก้ปัญหาการไล่เลื่อน trackbar แบบเดาสุ่มตอนแสง/สีจริงไม่ตรงกับค่า default
    """
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    frame = _latest_frame["frame"]
    if frame is None:
        return

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = [int(c) for c in hsv[y, x]]
    print(f"[PICKED] pixel ({x},{y}) -> H={h} S={s} V={v}")

    h_min = max(0, h - H_MARGIN)
    h_max = min(179, h + H_MARGIN)
    s_min = max(0, s - S_MARGIN)
    v_min = max(0, v - V_MARGIN)

    cv2.setTrackbarPos('H_MIN', 'Calibration', h_min)
    cv2.setTrackbarPos('H_MAX', 'Calibration', h_max)
    cv2.setTrackbarPos('S_MIN', 'Calibration', s_min)
    cv2.setTrackbarPos('S_MAX', 'Calibration', 255)
    cv2.setTrackbarPos('V_MIN', 'Calibration', v_min)
    cv2.setTrackbarPos('V_MAX', 'Calibration', 255)
    print(f"[AUTO-SET] H:{h_min}-{h_max}  S:{s_min}-255  V:{v_min}-255"
          f"  (ถ้า mask ยังไม่ขาวพอ ลองคลิกซ้ำที่จุดอื่นของก้อนเดียวกัน หรือลด/เพิ่ม MARGIN ในโค้ด)")

def main():
    load_config()
    cap = cv2.VideoCapture(0) # Change to 1 if using external webcam
    
    cv2.namedWindow('Calibration')
    
    colors = list(hsv_ranges.keys())
    current_color_idx = 0
    current_color = colors[current_color_idx]
    
    # Trackbar เลือกสีที่กำลังจูน (แทน/เสริมการกด 'n') ลากเลื่อนเพื่อสลับสีได้เลย
    cv2.createTrackbar('COLOR', 'Calibration', 0, len(colors) - 1, nothing)
    cv2.createTrackbar('H_MIN', 'Calibration', 0, 179, nothing)
    cv2.createTrackbar('S_MIN', 'Calibration', 0, 255, nothing)
    cv2.createTrackbar('V_MIN', 'Calibration', 0, 255, nothing)
    cv2.createTrackbar('H_MAX', 'Calibration', 179, 179, nothing)
    cv2.createTrackbar('S_MAX', 'Calibration', 255, 255, nothing)
    cv2.createTrackbar('V_MAX', 'Calibration', 255, 255, nothing)

    def update_trackbars(color):
        cv2.setTrackbarPos('H_MIN', 'Calibration', hsv_ranges[color]['lower'][0])
        cv2.setTrackbarPos('S_MIN', 'Calibration', hsv_ranges[color]['lower'][1])
        cv2.setTrackbarPos('V_MIN', 'Calibration', hsv_ranges[color]['lower'][2])
        cv2.setTrackbarPos('H_MAX', 'Calibration', hsv_ranges[color]['upper'][0])
        cv2.setTrackbarPos('S_MAX', 'Calibration', hsv_ranges[color]['upper'][1])
        cv2.setTrackbarPos('V_MAX', 'Calibration', hsv_ranges[color]['upper'][2])

    update_trackbars(current_color)

    print("=== HSV Calibration Tool ===")
    print("Drag the 'COLOR' slider (or press 'n') to switch color")
    print("Press 's' to SAVE config")
    print("Press 'q' to QUIT")
    print("Click anywhere on the 'Original' window to auto-set HSV range from that pixel")

    cv2.namedWindow('Original')
    cv2.setMouseCallback('Original', on_mouse_click)

    while True:
        ret, frame = cap.read()
        if not ret: break
        
        frame = cv2.resize(frame, (640, 480))
        _latest_frame["frame"] = frame
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # เช็คว่ามีการลาก trackbar 'COLOR' เปลี่ยนสีที่จูนอยู่หรือไม่
        color_idx = cv2.getTrackbarPos('COLOR', 'Calibration')
        if color_idx != current_color_idx:
            current_color_idx = color_idx
            current_color = colors[current_color_idx]
            update_trackbars(current_color)  # โหลดค่า H/S/V ที่เคยจูนไว้ของสีใหม่ขึ้น trackbar
            print(f"Switched to tuning: {current_color}")

        # Get values from trackbars
        h_min = cv2.getTrackbarPos('H_MIN', 'Calibration')
        s_min = cv2.getTrackbarPos('S_MIN', 'Calibration')
        v_min = cv2.getTrackbarPos('V_MIN', 'Calibration')
        h_max = cv2.getTrackbarPos('H_MAX', 'Calibration')
        s_max = cv2.getTrackbarPos('S_MAX', 'Calibration')
        v_max = cv2.getTrackbarPos('V_MAX', 'Calibration')

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        
        # Update current color dict
        hsv_ranges[current_color]['lower'] = [h_min, s_min, v_min]
        hsv_ranges[current_color]['upper'] = [h_max, s_max, v_max]

        mask = cv2.inRange(hsv, lower, upper)
        res = cv2.bitwise_and(frame, frame, mask=mask)

        # Display text info
        cv2.putText(frame, f"Tuning: {current_color.upper()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, "Press 'n' next, 's' save, 'q' quit", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(frame, "Or drag COLOR slider on Calibration window", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        cv2.putText(frame, "Click a stone/zone below to auto-pick its HSV", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

        cv2.imshow('Original', frame)
        cv2.imshow('Mask', mask)
        cv2.imshow('Result', res)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            save_config()
        elif key == ord('n'):
            next_idx = (current_color_idx + 1) % len(colors)
            cv2.setTrackbarPos('COLOR', 'Calibration', next_idx)  # การสลับจริงเกิดที่ต้น loop รอบถัดไป

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
