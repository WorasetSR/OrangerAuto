import websocket
import json
import time

WS_URL = "ws://192.168.4.1:81"

def test_servo():
    print(f"Connecting to ESP32 at {WS_URL} ...")
    try:
        ws = websocket.create_connection(WS_URL, timeout=3.0)
        print("✅ Connected!")
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return

    print("\n=== Servo Test Tool ===")
    print("พิมพ์ขนาดความกว้างหินเป็นมิลลิเมตร (เช่น 50, 80, 120)")
    print("พิมพ์ 'close'   เพื่อหุบประตู")
    print("พิมพ์ 'release' เพื่ออ้าประตูจนสุด (สำหรับปล่อยหิน)")
    print("พิมพ์ 'q'       เพื่อออกโปรแกรม\n")

    while True:
        cmd = input("ใส่คำสั่ง: ").strip()
        if cmd.lower() == 'q':
            break
            
        payload = {"vL": 0, "vR": 0}
        
        if cmd.lower() in ['close', 'release']:
            payload["door_cmd"] = cmd.lower()
        elif cmd.isdigit():
            # ถ้าพิมพ์ตัวเลขเพียวๆ ให้สร้างเป็นคำสั่งเปิดตามความกว้าง (บวกเผื่อ 20mm ตามใน robot_logic)
            # หรือจะส่งตัวเลขเพียวๆ ก็ได้ แต่ใน robot_logic.py มีการบวก 20 ไว้แล้ว
            # เราจะส่งไปตรงๆ ตามที่กรอกเลย เพื่อเทสกลไกเซอร์โว
            payload["door_cmd"] = f"open:{cmd}"
        else:
            print("❌ คำสั่งไม่ถูกต้อง กรุณาใส่ตัวเลข หรือ close / release")
            continue
            
        print(f"🚀 กำลังส่ง: {payload}")
        try:
            ws.send(json.dumps(payload))
            print("✅ ส่งสำเร็จ!")
        except Exception as e:
            print(f"❌ เกิดข้อผิดพลาดในการส่ง: {e}")
            break
            
    ws.close()
    print("ปิดการเชื่อมต่อแล้ว")

if __name__ == "__main__":
    test_servo()
