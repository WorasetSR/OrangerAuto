import cv2
import numpy as np
import json
import os

# รหัสกล้อง (0 = กล้องโน้ตบุ๊ก, 1 = กล้องต่อแยก)
CAMERA_IDX = 0

def main():
    print("=== เริ่มกระบวนการ Calibrate ทั้งระบบ ===")
    cap = cv2.VideoCapture(CAMERA_IDX)
    if not cap.isOpened():
        print("เกิดข้อผิดพลาดในการเปิดกล้อง โปรดตรวจสอบรหัส CAMERA_IDX")
        return

    # =========================================================================
    # STEP 1: Camera Calibration (Homography / มุมสนาม)
    # =========================================================================
    print("\n--- STEP 1: Camera Calibration (ตั้งค่ามุมสนาม) ---")
    clicked_points = []
    def click_event_cam(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked_points.append([x, y])
            print(f"คลิกจุดที่ {len(clicked_points)}: พิกัด (X={x}, Y={y})")

    cv2.namedWindow("1. Camera Calibration")
    cv2.setMouseCallback("1. Camera Calibration", click_event_cam)

    print("คำแนะนำ:")
    print("1. โปรดคลิกที่ 'มุมสนาม' ทั้ง 4 มุม ตามลำดับ: [1] ซ้ายบน [2] ขวาบน [3] ซ้ายล่าง [4] ขวาล่าง")
    print("2. กด 'r' เพื่อรีเซ็ต")
    print("3. เมื่อคลิกครบ 4 จุดแล้ว ให้กด 'c' เพื่อไปขั้นตอนถัดไป (หรือกด 'q' เพื่อออก)")

    while True:
        ret, frame = cap.read()
        if not ret: break
        frame = cv2.resize(frame, (640, 480))
        
        for i, pt in enumerate(clicked_points):
            cv2.circle(frame, (pt[0], pt[1]), 5, (0, 255, 0), -1)
            cv2.putText(frame, str(i+1), (pt[0]+10, pt[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
        cv2.imshow("1. Camera Calibration", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("ยกเลิกการทำงาน")
            cap.release()
            cv2.destroyAllWindows()
            return
        elif key == ord('r'):
            clicked_points.clear()
            print("🔄 รีเซ็ตจุดที่คลิกแล้ว")
        elif key == ord('c') and len(clicked_points) == 4:
            break

    cv2.destroyWindow("1. Camera Calibration")

    with open("calibration.json", "w") as f:
        json.dump({"src_pts": clicked_points}, f, indent=4)
    print(f"✅ บันทึก calibration.json เรียบร้อย")

    # =========================================================================
    # STEP 2: HSV Calibration (จูนสี)
    # =========================================================================
    print("\n--- STEP 2: HSV Calibration (จูนสี) ---")
    CONFIG_FILE = "hsv_config.json"
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
            hsv_ranges = json.load(f)
    else:
        hsv_ranges = {
            "red": {"lower": [168,175,181], "upper": [179,255,255]},
            "orange": {"lower": [9,170,150], "upper": [16,255,255]},
            "yellow": {"lower": [26,100,100], "upper": [35,255,255]},
            "green": {"lower": [51,120,124], "upper": [76,255,255]},
            "cyan": {"lower": [83,171,178], "upper": [103,251,255]},
            "blue": {"lower": [75,195,40], "upper": [140,255,143]},
            "purple": {"lower": [143,160,48], "upper": [163,255,255]}
        }

    colors = list(hsv_ranges.keys())
    current_color_idx = 0
    current_color = colors[current_color_idx]
    _latest_frame = {"frame": None}

    def nothing(x): pass

    cv2.namedWindow('2. HSV Tuning')
    cv2.createTrackbar('COLOR', '2. HSV Tuning', 0, len(colors) - 1, nothing)
    cv2.createTrackbar('H_MIN', '2. HSV Tuning', 0, 179, nothing)
    cv2.createTrackbar('S_MIN', '2. HSV Tuning', 0, 255, nothing)
    cv2.createTrackbar('V_MIN', '2. HSV Tuning', 0, 255, nothing)
    cv2.createTrackbar('H_MAX', '2. HSV Tuning', 179, 179, nothing)
    cv2.createTrackbar('S_MAX', '2. HSV Tuning', 255, 255, nothing)
    cv2.createTrackbar('V_MAX', '2. HSV Tuning', 255, 255, nothing)

    def update_trackbars(color):
        cv2.setTrackbarPos('H_MIN', '2. HSV Tuning', hsv_ranges[color]['lower'][0])
        cv2.setTrackbarPos('S_MIN', '2. HSV Tuning', hsv_ranges[color]['lower'][1])
        cv2.setTrackbarPos('V_MIN', '2. HSV Tuning', hsv_ranges[color]['lower'][2])
        cv2.setTrackbarPos('H_MAX', '2. HSV Tuning', hsv_ranges[color]['upper'][0])
        cv2.setTrackbarPos('S_MAX', '2. HSV Tuning', hsv_ranges[color]['upper'][1])
        cv2.setTrackbarPos('V_MAX', '2. HSV Tuning', hsv_ranges[color]['upper'][2])

    update_trackbars(current_color)

    def on_mouse_click_hsv(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN: return
        frame = _latest_frame["frame"]
        if frame is None: return
        hsv_px = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[y, x]
        h, s, v = int(hsv_px[0]), int(hsv_px[1]), int(hsv_px[2])
        H_MARGIN, S_MARGIN, V_MARGIN = 10, 60, 60
        cv2.setTrackbarPos('H_MIN', '2. HSV Tuning', max(0, h - H_MARGIN))
        cv2.setTrackbarPos('H_MAX', '2. HSV Tuning', min(179, h + H_MARGIN))
        cv2.setTrackbarPos('S_MIN', '2. HSV Tuning', max(0, s - S_MARGIN))
        cv2.setTrackbarPos('S_MAX', '2. HSV Tuning', 255)
        cv2.setTrackbarPos('V_MIN', '2. HSV Tuning', max(0, v - V_MARGIN))
        cv2.setTrackbarPos('V_MAX', '2. HSV Tuning', 255)

    cv2.namedWindow('2. HSV Original')
    cv2.setMouseCallback('2. HSV Original', on_mouse_click_hsv)

    print("คำแนะนำ:")
    print("- ลาก slider 'COLOR' หรือกด 'n' เพื่อเปลี่ยนสี")
    print("- คลิกซ้ายบนภาพเพื่อดูดสีอัตโนมัติ")
    print("- กด 's' บันทึก, 'c' ไปต่อ (หรือ 'q' เพื่อออก)")

    while True:
        ret, frame = cap.read()
        if not ret: break
        frame = cv2.resize(frame, (640, 480))
        _latest_frame["frame"] = frame
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        color_idx = cv2.getTrackbarPos('COLOR', '2. HSV Tuning')
        if color_idx != current_color_idx:
            current_color_idx = color_idx
            current_color = colors[current_color_idx]
            update_trackbars(current_color)

        h_min = cv2.getTrackbarPos('H_MIN', '2. HSV Tuning')
        s_min = cv2.getTrackbarPos('S_MIN', '2. HSV Tuning')
        v_min = cv2.getTrackbarPos('V_MIN', '2. HSV Tuning')
        h_max = cv2.getTrackbarPos('H_MAX', '2. HSV Tuning')
        s_max = cv2.getTrackbarPos('S_MAX', '2. HSV Tuning')
        v_max = cv2.getTrackbarPos('V_MAX', '2. HSV Tuning')

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        
        hsv_ranges[current_color]['lower'] = [h_min, s_min, v_min]
        hsv_ranges[current_color]['upper'] = [h_max, s_max, v_max]

        mask = cv2.inRange(hsv, lower, upper)
        res = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.putText(frame, f"Tuning: {current_color.upper()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, "Press 'n' next, 's' save, 'c' continue", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow('2. HSV Original', frame)
        cv2.imshow('2. HSV Mask', mask)
        cv2.imshow('2. HSV Result', res)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("ยกเลิกการทำงาน")
            cap.release()
            cv2.destroyAllWindows()
            return
        elif key == ord('s'):
            with open(CONFIG_FILE, 'w') as f:
                json.dump(hsv_ranges, f, indent=4)
            print(f"✅ บันทึกสีลง {CONFIG_FILE} แล้ว")
        elif key == ord('n'):
            next_idx = (current_color_idx + 1) % len(colors)
            cv2.setTrackbarPos('COLOR', '2. HSV Tuning', next_idx)
        elif key == ord('c'):
            with open(CONFIG_FILE, 'w') as f:
                json.dump(hsv_ranges, f, indent=4)
            break

    cv2.destroyWindow('2. HSV Original')
    cv2.destroyWindow('2. HSV Mask')
    cv2.destroyWindow('2. HSV Result')
    cv2.destroyWindow('2. HSV Tuning')

    # =========================================================================
    # STEP 3: Zone Setup (ตั้งค่า Danger Zone & โซนส่งหิน)
    # =========================================================================
    print("\n--- STEP 3: Zone Setup (ตั้งค่า Danger Zone & โซนส่งหิน) ---")
    
    FIELD_W_MM, FIELD_H_MM = 2100, 1200
    dst_pts = [[0, 0], [FIELD_W_MM, 0], [0, FIELD_H_MM], [FIELD_W_MM, FIELD_H_MM]]
    H_matrix, _ = cv2.findHomography(
        np.array(clicked_points, dtype=np.float32),
        np.array(dst_pts, dtype=np.float32)
    )
    inv_H = np.linalg.inv(H_matrix)

    def px_to_mm(px_pos):
        pt = np.array([[[px_pos[0], px_pos[1]]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(pt, H_matrix)
        return (int(dst[0][0][0]), int(dst[0][0][1]))

    setup_steps = ['Danger Zone (Top-Left)', 'Danger Zone (Bottom-Right)', 'red', 'blue', 'green', 'purple', 'cyan', 'orange']
    current_step_idx = 0
    
    manual_data = {
        "danger_zone": {},
        "center": [],
        "drop_zones": {}
    }
    temp_danger_pts = []
    
    cv2.namedWindow("3. Click Zones")
    
    def mouse_callback_dz(event, x, y, flags, param):
        nonlocal current_step_idx, temp_danger_pts
        if event == cv2.EVENT_LBUTTONDOWN:
            if current_step_idx < len(setup_steps):
                step_name = setup_steps[current_step_idx]
                pos_mm = px_to_mm((x, y))
                
                if step_name.startswith('Danger Zone'):
                    temp_danger_pts.append(pos_mm)
                    print(f"✅ บันทึก {step_name} ที่พิกัด (MM): {pos_mm}")
                    
                    if len(temp_danger_pts) == 2:
                        p1, p2 = temp_danger_pts
                        x_min, x_max = min(p1[0], p2[0]), max(p1[0], p2[0])
                        y_min, y_max = min(p1[1], p2[1]), max(p1[1], p2[1])
                        
                        manual_data["danger_zone"] = {
                            "x_min": x_min, "x_max": x_max,
                            "y_min": y_min, "y_max": y_max
                        }
                        manual_data["center"] = [(x_min + x_max)/2, (y_min + y_max)/2]
                        print(f"🎯 คำนวณจุดศูนย์กลางได้ที่: {manual_data['center']}")
                else:
                    manual_data["drop_zones"][step_name] = pos_mm
                    print(f"✅ บันทึก Drop Zone สี {step_name} ที่พิกัด (MM): {pos_mm}")
                
                current_step_idx += 1

    cv2.setMouseCallback("3. Click Zones", mouse_callback_dz)

    print("👉 คลิกเมาส์บนหน้าจอตามลำดับด้านล่างนี้:")
    for i, step in enumerate(setup_steps):
        print(f"   {i+1}. {step}")
    print("กด 'r' เพื่อรีเซ็ต, เมื่อครบกด 's' เพื่อบันทึกและจบการทำงาน")

    while True:
        ret, frame = cap.read()
        if not ret: break
        frame = cv2.resize(frame, (640, 480))
        
        # วาดโซนที่ตั้งค่าไปแล้ว
        for i, pos_mm in enumerate(temp_danger_pts):
            pt = np.array([[[pos_mm[0], pos_mm[1]]]], dtype=np.float32)
            dst = cv2.perspectiveTransform(pt, inv_H)
            px = (int(dst[0][0][0]), int(dst[0][0][1]))
            cv2.circle(frame, px, 8, (0, 255, 255), -1)
            if i == 1: # วาดกล่อง Danger Zone
                p1_px = cv2.perspectiveTransform(np.array([[[temp_danger_pts[0][0], temp_danger_pts[0][1]]]], dtype=np.float32), inv_H)[0][0]
                cv2.rectangle(frame, (int(p1_px[0]), int(p1_px[1])), px, (0, 255, 255), 2)
                
        for c, pos_mm in manual_data["drop_zones"].items():
            pt = np.array([[[pos_mm[0], pos_mm[1]]]], dtype=np.float32)
            dst = cv2.perspectiveTransform(pt, inv_H)
            px = (int(dst[0][0][0]), int(dst[0][0][1]))
            cv2.circle(frame, px, 8, (0, 255, 0), -1)
            cv2.putText(frame, c, (px[0]+10, px[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        if current_step_idx < len(setup_steps):
            target_step = setup_steps[current_step_idx]
            msg = f"Click for: {target_step}"
            cv2.putText(frame, msg, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        else:
            cv2.putText(frame, "ALL ZONES SET! Press 's' to Save & Finish", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
            cv2.putText(frame, "Or 'r' to Reset", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
        cv2.imshow("3. Click Zones", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and current_step_idx == len(setup_steps):
            with open("manual_dropzones.json", "w") as f:
                json.dump(manual_data, f, indent=4)
            print("✅ บันทึก manual_dropzones.json เรียบร้อยแล้ว!")
            break
        elif key == ord('r'):
            print("🔄 รีเซ็ตการตั้งค่า เริ่มใหม่ทั้งหมด")
            manual_data["drop_zones"].clear()
            temp_danger_pts.clear()
            current_step_idx = 0
        elif key == ord('q'):
            break

    print("\n🎉 === เสร็จสิ้นการ Calibrate ทั้งระบบ! ===\n")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
