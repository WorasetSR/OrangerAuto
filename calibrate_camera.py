import cv2
import numpy as np

# รายการเก็บพิกัดที่คลิก
clicked_points = []

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_points.append([x, y])
        print(f"คลิกจุดที่ {len(clicked_points)}: พิกัด (X={x}, Y={y})")

def main():
    cap = cv2.VideoCapture(0)
    
    cv2.namedWindow("Camera Calibration")
    # เปิดรับการคลิกเมาส์
    cv2.setMouseCallback("Camera Calibration", click_event)
    
    print("========================================")
    print("คำแนะนำ:")
    print("1. โปรดคลิกที่ 'มุมสนาม' ทั้ง 4 มุม ตามลำดับดังนี้:")
    print("   [1] มุมซ้ายบน  [2] มุมขวาบน  [3] มุมซ้ายล่าง  [4] มุมขวาล่าง")
    print("2. หากคลิกผิด ให้กดปุ่ม 'r' เพื่อรีเซ็ตค่าใหม่")
    print("3. หากคลิกครบ 4 มุมแล้ว ให้กดปุ่ม 'q' เพื่อออกและดูผลลัพธ์")
    print("========================================")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("เกิดข้อผิดพลาดในการเปิดกล้อง")
            break
            
        frame = cv2.resize(frame, (640, 480))
        
        # วาดวงกลมทับจุดที่คลิกไปแล้ว
        for i, pt in enumerate(clicked_points):
            cv2.circle(frame, (pt[0], pt[1]), 5, (0, 255, 0), -1)
            cv2.putText(frame, str(i+1), (pt[0]+10, pt[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
        cv2.imshow("Camera Calibration", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            clicked_points.clear()
            print("รีเซ็ตจุดที่คลิกแล้ว กรุณาเริ่มคลิกใหม่")

    cap.release()
    cv2.destroyAllWindows()
    
    if len(clicked_points) == 4:
        print("\n=== ได้พิกัดครบ 4 มุมแล้ว! นำค่าด้านล่างนี้ไปใส่ใน vision_tracker.py ได้เลย ===")
        print(f"src_pts = {clicked_points}")
    else:
        print(f"\nคุณคลิกไปแค่ {len(clicked_points)} จุด (ต้องคลิกให้ครบ 4 จุดแล้วค่อยกด q)")

if __name__ == "__main__":
    main()
