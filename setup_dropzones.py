import cv2
import json
import os
import numpy as np
from vision_tracker import VisionTracker

def main():
    print("กำลังเปิดกล้องสำหรับตั้งค่า Drop Zone ด้วยมือ (Manual Setup)...")
    tracker = VisionTracker()
    
    colors_to_set = ['red', 'blue', 'green', 'purple', 'cyan', 'orange']
    current_color_idx = 0
    manual_drop_zones = {}
    
    cv2.namedWindow("Click Drop Zones")
    
    def mouse_callback(event, x, y, flags, param):
        nonlocal current_color_idx
        if event == cv2.EVENT_LBUTTONDOWN:
            if current_color_idx < len(colors_to_set):
                color = colors_to_set[current_color_idx]
                # แปลงพิกัดพิกเซลบนหน้าจอ เป็นมิลลิเมตร (MM) ของสนามจริง
                pos_mm = tracker.px_to_mm((x, y))
                manual_drop_zones[color] = pos_mm
                print(f"✅ บันทึก Drop Zone สี {color} ที่พิกัด (MM): {pos_mm}")
                current_color_idx += 1

    cv2.setMouseCallback("Click Drop Zones", mouse_callback)

    print("\n========================================")
    print("👉 วิธีใช้: ให้คลิกเมาส์บนหน้าจอที่ตำแหน่ง Drop Zone ตามลำดับสีด้านล่างนี้:")
    for i, c in enumerate(colors_to_set):
        print(f"   {i+1}. {c.upper()}")
    
    # คำนวณ Inverse Homography ก่อนเข้าลูปเพื่อลดภาระ CPU
    inv_H = np.linalg.inv(tracker.H_matrix)
    
    while True:
        ret, frame = tracker.cap.read()
        if not ret:
            break
            
        frame = cv2.resize(frame, (640, 480))
        
        # วาดโซนที่ตั้งค่าไปแล้ว
        for c, pos_mm in manual_drop_zones.items():
            # แปลงกลับเป็น px เพื่อวาดโชว์
            pt = np.array([[[pos_mm[0], pos_mm[1]]]], dtype=np.float32)
            dst = cv2.perspectiveTransform(pt, inv_H)
            px = (int(dst[0][0][0]), int(dst[0][0][1]))
            cv2.circle(frame, px, 8, (0, 255, 0), -1)
            cv2.putText(frame, c, (px[0]+10, px[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        if current_color_idx < len(colors_to_set):
            target_color = colors_to_set[current_color_idx]
            msg = f"Click for: {target_color.upper()}"
            cv2.putText(frame, msg, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        else:
            cv2.putText(frame, "ALL ZONES SET! Press 's' to Save & Exit", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
            cv2.putText(frame, "Or 'r' to Reset", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
        cv2.imshow("Click Drop Zones", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and current_color_idx == len(colors_to_set):
            with open("manual_dropzones.json", "w") as f:
                json.dump(manual_drop_zones, f, indent=4)
            print("💾 บันทึกค่าลงไฟล์ manual_dropzones.json เรียบร้อยแล้ว!")
            break
        elif key == ord('r'):
            print("🔄 รีเซ็ตการตั้งค่า เริ่มใหม่ตั้งแต่สี Red")
            manual_drop_zones.clear()
            current_color_idx = 0
        elif key == ord('q'):
            break

    tracker.close()

if __name__ == "__main__":
    main()
