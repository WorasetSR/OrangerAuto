"""
สคริปต์ทดสอบมอเตอร์ตรงๆ ไม่ผ่านกล้อง/planner ใดๆ ทั้งสิ้น
ใช้แยกปัญหาว่าอยู่ที่ python logic หรือฮาร์ดแวร์/เฟิร์มแวร์ ESP32

วิธีใช้: รันแล้วสังเกตพฤติกรรมจริงของรถทีละคำสั่ง (มีหน่วงเวลาให้ดูทัน)
"""
import time
import json
import websocket

HOST = "ws://192.168.4.1:81"

def send(ws, vL, vR, label=""):
    payload = {"vL": int(vL), "vR": int(vR)}
    print(f">>> ส่งคำสั่ง: vL={vL:4d}  vR={vR:4d}   {label}")
    ws.send(json.dumps(payload))

def main():
    print(f"กำลังเชื่อมต่อ {HOST} ...")
    ws = websocket.WebSocket()
    ws.connect(HOST, timeout=3)
    print("เชื่อมต่อสำเร็จ!\n")

    tests = [
        (0,    0,    "หยุด (baseline)"),
        (150,  150,  "ควรวิ่งตรงไปข้างหน้า"),
        (0,    0,    "หยุด"),
        (-150, -150, "ควรถอยหลังตรงๆ"),
        (0,    0,    "หยุด"),
        (150,  -150, "ควรหมุนขวาอยู่กับที่ (ตามสูตรในโค้ด error>0)"),
        (0,    0,    "หยุด"),
        (-150, 150,  "ควรหมุนซ้ายอยู่กับที่"),
        (0,    0,    "หยุด"),
    ]

    for vL, vR, label in tests:
        send(ws, vL, vR, label)
        time.sleep(2.0)  # ดูผลจริง 2 วิ ต่อคำสั่ง ปรับเวลาได้ตามสะดวก

    ws.close()
    print("\nทดสอบเสร็จสิ้น ปิดการเชื่อมต่อแล้ว")

if __name__ == "__main__":
    main()
