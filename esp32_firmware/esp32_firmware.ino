#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "DoorControl.h"

// ==========================================
// 1. กำหนดพินการเชื่อมต่อฮาร์ดแวร์ (Hardware Pins)
// ==========================================
// มอเตอร์ DC (ซ้าย) แบบ 2 เส้น (เสียบ IN1, IN2)
#define L_IN1_PIN 27
#define L_IN2_PIN 26

// มอเตอร์ DC (ขวา) แบบ 2 เส้น (เสียบ IN1, IN2)
#define R_IN1_PIN 17
#define R_IN2_PIN 16

// Servo ประตู (ซ้าย-ขวา)
#define SERVO_L_PIN 33
#define SERVO_R_PIN 19

// การตั้งค่า PWM สำหรับ ESP32 Native (ledc) 
// (มอเตอร์ 2 ตัว แบบ 2 เส้น ต้องใช้ 4 ช่อง PWM)
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define L_IN1_CH 0
#define L_IN2_CH 1
#define R_IN1_CH 2
#define R_IN2_CH 3

// ==========================================
// 2. ตั้งค่าเครือข่าย Wi-Fi
// ==========================================
const char* ssid = "Oranger";       // ตั้งชื่อ Wi-Fi ของหุ่นยนต์
const char* password = "Lnwza007";      // ⚠️ ควรเปลี่ยนก่อนแข่งจริง

WebSocketsServer webSocket = WebSocketsServer(81); // Port 81 ตาม Blueprint

// ออบเจ็กต์ควบคุม Servo ถูกจัดการโดย DoorControl แล้ว

// ตัวแปรสำหรับระบบ Safety Timeout
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 500; // หากไม่ได้รับคำสั่งเกิน 500ms ให้หยุดรถ
bool commandReceived = false;         // Flag ป้องกัน timeout spam ตอนเปิดเครื่อง

// ==========================================
// ฟังก์ชันควบคุมมอเตอร์ (ใช้ ledcWrite แทน analogWrite)
// ==========================================
// speed มีค่าตั้งแต่ -255 ถึง 255 (บวก = เดินหน้า, ลบ = ถอยหลัง)
void setMotorLeft(int speed) {
  int magnitude = constrain(abs(speed), 0, 255);
  if (speed == 0) {
    ledcWrite(L_IN1_CH, 0);
    ledcWrite(L_IN2_CH, 0);
  } else if (speed > 0) {
    ledcWrite(L_IN1_CH, magnitude);
    ledcWrite(L_IN2_CH, 0);
  } else {
    ledcWrite(L_IN1_CH, 0);
    ledcWrite(L_IN2_CH, magnitude);
  }
}

void setMotorRight(int speed) {
  int magnitude = constrain(abs(speed), 0, 255);
  if (speed == 0) {
    ledcWrite(R_IN1_CH, 0);
    ledcWrite(R_IN2_CH, 0);
  } else if (speed > 0) {
    ledcWrite(R_IN1_CH, magnitude);
    ledcWrite(R_IN2_CH, 0);
  } else {
    ledcWrite(R_IN1_CH, 0);
    ledcWrite(R_IN2_CH, magnitude);
  }
}

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
}

// ==========================================
// ฟังก์ชันจัดการ WebSocket Events
// ==========================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      stopMotors(); // หยุดรถเมื่อหลุดการเชื่อมต่อ
      commandReceived = false;
      break;
      
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      break;
    }
    
    case WStype_TEXT: {
      // เมื่อได้รับข้อความ JSON จาก Python
      lastCommandTime = millis(); // รีเซ็ตเวลา Timeout
      commandReceived = true;     // ยืนยันว่ามีการติดต่อแล้ว
      
      // ตัวอย่าง Payload: {"vL": 180, "vR": -180, "doorL": 30, "doorR": 150}
      StaticJsonDocument<200> doc; // หากใช้ ArduinoJson v7 อาจต้องเปลี่ยนเป็น JsonDocument doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      // ดึงค่าคำสั่ง (ถ้าไม่มีค่าส่งมา ให้ใช้ค่า default เป็น 0)
      int vL = doc["vL"] | 0;
      int vR = doc["vR"] | 0;

      // สั่งมอเตอร์
      setMotorLeft(vL);
      setMotorRight(vR);

      // ดึงค่าคำสั่งประตู
      if (doc.containsKey("door_cmd")) {
        const char* door_cmd = doc["door_cmd"];
        if (strcmp(door_cmd, "collect") == 0) {
          doorControl_manualOpenCollect();
        } else if (strcmp(door_cmd, "release") == 0) {
          doorControl_manualOpenFull();
        } else if (strcmp(door_cmd, "close") == 0) {
          doorControl_manualClose();
        }
      }
      
      break;
    }
  }
}

// ==========================================
// ฟังก์ชัน Setup
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // ตั้งค่า Native PWM (ledc) สำหรับมอเตอร์แบบ 2 เส้น (4 ขา)
  ledcSetup(L_IN1_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(L_IN1_PIN, L_IN1_CH);
  
  ledcSetup(L_IN2_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(L_IN2_PIN, L_IN2_CH);
  
  ledcSetup(R_IN1_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(R_IN1_PIN, R_IN1_CH);
  
  ledcSetup(R_IN2_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(R_IN2_PIN, R_IN2_CH);

  // เริ่มต้น DoorControl
  doorControl_begin(SERVO_L_PIN, SERVO_R_PIN);
  
  // ค่าเริ่มต้นปิดมอเตอร์
  stopMotors();

  // สร้าง Wi-Fi Access Point (โหมด SoftAP ปล่อย Wi-Fi เองให้คอมเชื่อม)
  Serial.println("Starting Access Point...");
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // เริ่มต้น WebSocket Server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  Serial.println("System Ready.");
}

// ==========================================
// ฟังก์ชัน Loop
// ==========================================
void loop() {
  // รันระบบ WebSocket เสมอ
  webSocket.loop();
  
  // รันระบบ State Machine ของประตู
  doorControl_update();
  
  // ระบบ Safety Timeout (ถ้าสัญญาณ Wi-Fi หาย ขาดช่วงเกิน 500ms)
  // เช็คเฉพาะเมื่อเคยมีการส่งคำสั่งมาแล้ว (commandReceived == true) ป้องกันการสแปมตอนเปิดเครื่อง
  if (commandReceived && (millis() - lastCommandTime > TIMEOUT_MS)) {
    stopMotors();
    doorControl_failSafe(); // ปิดประตูฉุกเฉิน
    commandReceived = false; // ปิด flag เพื่อไม่ให้เช็คซ้ำจนกว่าจะมีคำสั่งใหม่
  }
}
