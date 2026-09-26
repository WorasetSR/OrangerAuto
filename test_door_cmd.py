"""
ทดสอบว่าการเพิ่ม key "door_cmd" เข้าไปใน JSON ทำให้เฟิร์มแวร์ ESP32 พังหรือไม่
เทียบกับ payload แบบง่ายที่ยืนยันแล้วว่าใช้งานได้ (test_motors.py)
"""
import time
import json
import websocket

HOST = "ws://192.168.4.1:81"

def send(ws, payload, label=""):
    print(f">>> ส่ง: {json.dumps(payload)}   {label}")
    ws.send(json.dumps(payload))

def main():
    print(f"กำลังเชื่อมต่อ {HOST} ...")
    ws = websocket.WebSocket()
    ws.connect(HOST, timeout=3)
    print("เชื่อมต่อสำเร็จ!\n")

    print("=== ทดสอบที่ 1: payload แบบง่าย (ไม่มี door_cmd) — ควรวิ่งตรง ===")
    send(ws, {"vL": 150, "vR": 150}, "ไม่มี door_cmd")
    time.sleep(3)
    send(ws, {"vL": 0, "vR": 0}, "หยุด")
    time.sleep(1)

    print("\n=== ทดสอบที่ 2: payload แบบเดียวกับ DRIVE_INGEST (มี door_cmd) ===")
    send(ws, {"vL": 150, "vR": 150, "door_cmd": "collect"}, "มี door_cmd='collect'")
    time.sleep(3)
    send(ws, {"vL": 0, "vR": 0}, "หยุด")
    time.sleep(1)

    ws.close()
    print("\nทดสอบเสร็จสิ้น — เทียบดูว่าทดสอบที่ 1 กับ 2 รถวิ่งเหมือนกันไหม")
    print("ถ้าทดสอบที่ 1 วิ่งแต่ทดสอบที่ 2 ไม่วิ่ง (หรือมีเสียงจี๊ด) = door_cmd ทำให้ parse พัง")

if __name__ == "__main__":
    main()
