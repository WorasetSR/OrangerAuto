import cv2
import numpy as np
from vision_tracker import VisionTracker
from path_planner import PathPlanner


def mm_to_px(H_matrix, mm_pos):
    """ฟังก์ชันแปลงหน่วยมิลลิเมตร (MM) กลับเป็นพิกัดพิกเซลบนหน้าจอ (Pixel)"""
    inv_H = np.linalg.inv(H_matrix)
    pt = np.array([[[mm_pos[0], mm_pos[1]]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(pt, inv_H)
    return (int(dst[0][0][0]), int(dst[0][0][1]))

def main():
    print("กำลังเปิดกล้องและโหลดระบบ...")
    tracker = VisionTracker()  # calibration.json is loaded automatically inside __init__
    planner = PathPlanner()
    
    cv2.namedWindow("Debug Vision (Monitor)", cv2.WINDOW_NORMAL)
    
    print("========================================")
    print("✅ Debug Vision ทำงานแล้ว!")
    print("กดปุ่ม 'q' ที่คีย์บอร์ดเพื่อปิดหน้าต่างนี้")
    print("========================================")
    
    while True:
        state, frame = tracker.get_state()
        if frame is None:
            break
            
        if state is None:
            cv2.imshow("Debug Vision (Monitor)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue
            
        H = tracker.H_matrix
        
        # 1. วาดก้อนหิน (สีและข้อความ)
        for stone in state["stones"]:
            px_pos = mm_to_px(H, stone["pos"])
            color = stone["color"]
            # พิมพ์ข้อความสีหินกำกับไว้เหนือจุด
            cv2.putText(frame, color.upper(), (px_pos[0] + 10, px_pos[1] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                        
        # 2. วาด Danger Zone (โซนอันตรายสีแดง)
        center, danger_zone = tracker.calculate_dynamic_zones(state["stones"])
        if danger_zone is not None:
            # แปลงพิกัดโซนจาก MM กลับเป็นพิกเซลเพื่อวาดลงจอ
            pt1_mm = (danger_zone["x_min"], danger_zone["y_min"])
            pt2_mm = (danger_zone["x_max"], danger_zone["y_max"])
            pt1_px = mm_to_px(H, pt1_mm)
            pt2_px = mm_to_px(H, pt2_mm)
            # วาดกรอบสี่เหลี่ยมสีแดง
            cv2.rectangle(frame, pt1_px, pt2_px, (0, 0, 255), 2)
            cv2.putText(frame, "DANGER ZONE", (pt1_px[0], pt1_px[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        
        # 3. วาดรถหุ่นยนต์และ Waypoint 
        robot = state["robot"]
        if robot["pos"] is not None and robot["mouth_pos"] is not None:
            r_px = mm_to_px(H, robot["pos"])
            m_px = mm_to_px(H, robot["mouth_pos"])
            
            # วาดเส้นทิศทาง (Heading) จากกลางรถไปที่ปาก
            cv2.line(frame, r_px, m_px, (0, 255, 0), 2)
            # วาดจุดสีแดงระบุตำแหน่งปากงับหิน
            cv2.circle(frame, m_px, 6, (0, 0, 255), -1) 
            cv2.putText(frame, "ROBOT", (r_px[0] - 20, r_px[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, "MOUTH", (m_px[0] + 10, m_px[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        
            # ลองสร้างเส้นทางสมมติไปยังหินก้อนแรก เพื่อแสดงการทำงานของ Waypoint
            if state["stones"] and danger_zone is not None:
                planner.update_danger_zone(
                    danger_zone["x_min"], danger_zone["x_max"],
                    danger_zone["y_min"], danger_zone["y_max"])
                target = state["stones"][0]["pos"]
                waypoints = planner.generate_perimeter_waypoints(robot["pos"], target)
                
                if waypoints:
                    prev_pt = r_px
                    for wp in waypoints:
                        wp_px = mm_to_px(H, wp)
                        # วาดเส้นทางสีม่วงแดง (Magenta)
                        cv2.line(frame, prev_pt, wp_px, (255, 0, 255), 2)
                        cv2.circle(frame, wp_px, 6, (255, 0, 255), -1)
                        prev_pt = wp_px
                    cv2.putText(frame, "PATH", (r_px[0] + 10, r_px[1] + 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                                
        # 4. วาด Drop Zones (ขอบเขตจริงรัศมี 8.5 ซม.)
        for color, pos_mm in state["drop_zones"].items():
            dz_px = mm_to_px(H, pos_mm)
            
            # วาดรัศมีวงกลมแสดงขอบเขต Drop Zone รัศมี 85 mm (8.5 ซม.)
            # คำนวณพิกัดขอบบนเพื่อหารัศมีในหน่วยพิกเซล
            edge_px = mm_to_px(H, (pos_mm[0] + 85, pos_mm[1]))
            radius_px = abs(edge_px[0] - dz_px[0])
            
            # ตีเส้นวงกลมและจุดศูนย์กลาง
            cv2.circle(frame, dz_px, radius_px, (255, 255, 0), 2)
            cv2.circle(frame, dz_px, 5, (255, 255, 0), -1)
            
            cv2.putText(frame, f"DROP: {color.upper()}", (dz_px[0]-40, dz_px[1]-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # สรุปสถิติที่มุมซ้ายบนจอ
        cv2.putText(frame, f"Stones visible: {len(state['stones'])}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Debug Vision (Monitor)", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    tracker.close()

if __name__ == "__main__":
    main()