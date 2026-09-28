import cv2
import json
import os
import numpy as np
from vision_tracker import VisionTracker

def main():
    print("กำลังเปิดกล้องสำหรับตั้งค่า Drop Zone ด้วยมือ (Manual Setup)...")
    tracker = VisionTracker(camera_idx=0)
    
    setup_steps = ['Danger Zone (Top-Left)', 'Danger Zone (Bottom-Right)', 'red', 'blue', 'green', 'purple', 'cyan', 'orange']
    current_step_idx = 0
    
    manual_data = {
        "danger_zone": {},
        "center": [],
        "drop_zones": {}
    }
    
    temp_danger_pts = []
    
    cv2.namedWindow("Click Drop Zones")
    
    def mouse_callback(event, x, y, flags, param):
        nonlocal current_step_idx, temp_danger_pts
        if event == cv2.EVENT_LBUTTONDOWN:
            if current_step_idx < len(setup_steps):
                step_name = setup_steps[current_step_idx]
                pos_mm = tracker.px_to_mm((x, y))
                
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
                        print(f"🎯 คำนวณจุดศูนย์กลางกองหินได้ที่: {manual_data['center']}")
                else:
                    manual_data["drop_zones"][step_name] = pos_mm
                    print(f"✅ บันทึก Drop Zone สี {step_name} ที่พิกัด (MM): {pos_mm}")
                
                current_step_idx += 1

    cv2.setMouseCallback("Click Drop Zones", mouse_callback)

    print("\n========================================")
    print("👉 วิธีใช้: ให้คลิกเมาส์บนหน้าจอตามลำดับด้านล่างนี้:")
    for i, step in enumerate(setup_steps):
        print(f"   {i+1}. {step}")
    
    # คำนวณ Inverse Homography ก่อนเข้าลูปเพื่อลดภาระ CPU
    inv_H = np.linalg.inv(tracker.H_matrix)
    
    while True:
        ret, frame = tracker.cap.read()
        if not ret:
            break
            
        frame = cv2.resize(frame, (640, 480))
        
        # วาดโซนที่ตั้งค่าไปแล้ว
        for i, pos_mm in enumerate(temp_danger_pts):
            pt = np.array([[[pos_mm[0], pos_mm[1]]]], dtype=np.float32)
            dst = cv2.perspectiveTransform(pt, inv_H)
            px = (int(dst[0][0][0]), int(dst[0][0][1]))
            cv2.circle(frame, px, 8, (0, 255, 255), -1)
            if i == 1: # วาดกล่อง
                p1_px = cv2.perspectiveTransform(np.array([[[temp_danger_pts[0][0], temp_danger_pts[0][1]]]], dtype=np.float32), inv_H)[0][0]
                cv2.rectangle(frame, (int(p1_px[0]), int(p1_px[1])), px, (0, 255, 255), 2)
                
        for c, pos_mm in manual_data["drop_zones"].items():
            # แปลงกลับเป็น px เพื่อวาดโชว์
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
            cv2.putText(frame, "ALL ZONES SET! Press 's' to Save & Exit", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
            cv2.putText(frame, "Or 'r' to Reset", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
        cv2.imshow("Click Drop Zones", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and current_step_idx == len(setup_steps):
            with open("manual_dropzones.json", "w") as f:
                json.dump(manual_data, f, indent=4)
            print("💾 บันทึกค่าลงไฟล์ manual_dropzones.json เรียบร้อยแล้ว!")
            break
        elif key == ord('r'):
            print("🔄 รีเซ็ตการตั้งค่า เริ่มใหม่ทั้งหมด")
            manual_data["drop_zones"].clear()
            temp_danger_pts.clear()
            current_step_idx = 0
        elif key == ord('q'):
            break

    tracker.close()

if __name__ == "__main__":
    main()
