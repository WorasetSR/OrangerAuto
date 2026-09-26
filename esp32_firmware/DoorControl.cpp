#include "DoorControl.h"
#include <ESP32Servo.h>

// ตัวแปรภายในไฟล์นี้เท่านั้น
static Servo servoLeft;
static Servo servoRight;

// มุมเซอร์โวตอนปิดสนิท (ต้อง Calibrate หน้างาน)
static const int ANGLE_LEFT_CLOSED  = 90;
static const int ANGLE_RIGHT_CLOSED = 90;

// ทิศทางการหมุนตอนเปิดเทียบกับตำแหน่งปิด: +1 = มุมเพิ่มขึ้น, -1 = มุมลดลง
static const int LEFT_OPEN_DIRECTION  = -1;
static const int RIGHT_OPEN_DIRECTION = +1;

// gap ตรงกลาง = CHAMBER_WIDTH_CM - 2 * DOOR_WIDTH_CM * cos(theta)
static const float CHAMBER_WIDTH_CM = 12.0;  // ความกว้างช่องเก็บทั้งหมด
static const float DOOR_WIDTH_CM    = 5.8;   // ความกว้างแต่ละบาน
static const float STONE_WIDTH_CM   = 5.0;   // ความกว้างก้อนหินใหญ่ที่สุด
static const float FULLOPEN_DEG     = 90.0;  // เปิดสุด

// มุมที่คำนวณได้ (เติมค่าใน doorControl_begin)
static int angleLeftCollect, angleRightCollect;
static int angleLeftFullOpen, angleRightFullOpen;

// สัดส่วนการเปิด (เทียบกับ FULLOPEN_DEG) ของท่า "ง้างเก็บหิน" เก็บไว้เผื่อ
// ต้อง sync กับ currentOpenFraction เวลาเรียก manualOpenCollect() / startCollect()
static float collectOpenFraction;

// เวลา (ปรับจากการทดสอบบนสนามแข่ง)
static const unsigned long COLLECT_HOLD_MS = 1200; // เวลาง้างค้างรอหินเข้ามาอยู่ในรัศมี
static const unsigned long MOVE_SETTLE_MS  = 300;  // เวลาประมาณที่เซอร์โวใช้เคลื่อนถึงมุมเป้าหมาย

// state machine
static DoorControlState state = DOOR_IDLE;
static unsigned long stateTimer = 0;

// สัดส่วนการเปิดปัจจุบัน 0.0 (ปิดสนิท) - 1.0 (เปิดสุด)
// อัปเดตทุกครั้งที่มีการสั่งขยับประตู ไม่ว่าจะมาจากฟังก์ชันไหน เพื่อให้
// doorControl_isClosed() / doorControl_isOpenFull() / doorControl_getOpenFraction() สื่อสถานะจริงเสมอ
// (ต่างจาก isIdle() ที่บอกแค่ว่า "ไม่มี autonomous sequence กำลังรัน" เท่านั้น)
static float currentOpenFraction = 0.0f;

// คำนวณมุมหมุนจากตำแหน่งปิด ให้ได้ gap เท่ากับ STONE_WIDTH_CM
static float computeCollectOpenDeg() {
  float cosTheta = (CHAMBER_WIDTH_CM - STONE_WIDTH_CM) / (2.0 * DOOR_WIDTH_CM);
  cosTheta = constrain(cosTheta, -1.0, 1.0);
  return degrees(acos(cosTheta));
}

static void setDoorAngles(int leftAngle, int rightAngle) {
  servoLeft.write(leftAngle);
  servoRight.write(rightAngle);
}

// ฟังก์ชัน public

void doorControl_begin(int pinLeft, int pinRight) {
  float collectDeg = computeCollectOpenDeg();
  angleLeftCollect   = ANGLE_LEFT_CLOSED  + LEFT_OPEN_DIRECTION  * collectDeg;
  angleRightCollect  = ANGLE_RIGHT_CLOSED + RIGHT_OPEN_DIRECTION * collectDeg;
  angleLeftFullOpen  = ANGLE_LEFT_CLOSED  + LEFT_OPEN_DIRECTION  * FULLOPEN_DEG;
  angleRightFullOpen = ANGLE_RIGHT_CLOSED + RIGHT_OPEN_DIRECTION * FULLOPEN_DEG;
  collectOpenFraction = collectDeg / FULLOPEN_DEG;

  Serial.print("[DoorControl] collectOpenDeg=");
  Serial.print(collectDeg);
  Serial.print(" angleLeftCollect=");
  Serial.print(angleLeftCollect);
  Serial.print(" angleRightCollect=");
  Serial.println(angleRightCollect);

  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoLeft.setPeriodHertz(50);
  servoRight.setPeriodHertz(50);
  servoLeft.attach(pinLeft, 500, 2400);
  servoRight.attach(pinRight, 500, 2400);

  doorControl_goHome();
}

bool doorControl_startCollect() {
  if (state != DOOR_IDLE) return false; // กันสั่งซ้อนระหว่างทำงานอยู่
  setDoorAngles(angleLeftCollect, angleRightCollect);
  currentOpenFraction = collectOpenFraction;
  state = DOOR_COLLECT_OPENING;
  stateTimer = millis();
  return true;
}

bool doorControl_startRelease() {
  if (state != DOOR_IDLE) return false;
  setDoorAngles(angleLeftFullOpen, angleRightFullOpen);
  currentOpenFraction = 1.0f;
  state = DOOR_RELEASE_OPENING;
  stateTimer = millis();
  return true;
}

void doorControl_goHome() {
  setDoorAngles(ANGLE_LEFT_CLOSED, ANGLE_RIGHT_CLOSED);
  currentOpenFraction = 0.0f;
  state = DOOR_IDLE;
}

// Manual (preset)
void doorControl_manualOpenCollect() {
  setDoorAngles(angleLeftCollect, angleRightCollect);
  currentOpenFraction = collectOpenFraction;
  state = DOOR_IDLE;
}

void doorControl_manualOpenFull() {
  setDoorAngles(angleLeftFullOpen, angleRightFullOpen);
  currentOpenFraction = 1.0f;
  state = DOOR_IDLE;
}

void doorControl_manualClose() {
  setDoorAngles(ANGLE_LEFT_CLOSED, ANGLE_RIGHT_CLOSED);
  currentOpenFraction = 0.0f;
  state = DOOR_IDLE;
}

// Manual (proportional) สำหรับ gesture ที่ส่งค่ามุมต่อเนื่อง
// ใช้ FULLOPEN_DEG และทิศทาง LEFT/RIGHT_OPEN_DIRECTION ชุดเดียวกับ preset
// เพื่อไม่ให้ทิศทางการหมุนขัดกันเวลาสลับไปมาระหว่าง preset กับ proportional
void doorControl_manualSetOpen(float openFraction) {
  if (isnan(openFraction)) return; // กันค่าพังจาก network/parse ผิด ไม่ขยับอะไรเลย
  openFraction = constrain(openFraction, 0.0f, 1.0f);

  int leftAngle  = ANGLE_LEFT_CLOSED  + LEFT_OPEN_DIRECTION  * (int)round(openFraction * FULLOPEN_DEG);
  int rightAngle = ANGLE_RIGHT_CLOSED + RIGHT_OPEN_DIRECTION * (int)round(openFraction * FULLOPEN_DEG);
  setDoorAngles(leftAngle, rightAngle);

  currentOpenFraction = openFraction;
  state = DOOR_IDLE; // เหมือน manual ตัวอื่น คือยกเลิกลำดับอัตโนมัติที่ค้างอยู่ทันที
}

// Fail-safe เรียกตอนขาดการติดต่อ/timeout ไม่ว่ากำลังอยู่สเตท/โหมดไหน
// ใช้ logic เดียวกับ manualClose() เพื่อไม่ให้มีพฤติกรรมสองแบบปนกันระหว่างปิดตอนปกติกับตอนฉุกเฉิน
void doorControl_failSafe() {
  doorControl_manualClose();
}

bool doorControl_isIdle() {
  return state == DOOR_IDLE;
}

// ต่างจาก isIdle() เช็กสถานะประตูจริง ไม่ใช่แค่ว่ามี autonomous sequence ค้างอยู่ไหม
// ใช้ threshold เล็กน้อย (2%) กัน float rounding แทนที่จะเทียบเท่ากับ 0.0f ตรงๆ
bool doorControl_isClosed() {
  return currentOpenFraction <= 0.02f;
}

// เช็กว่าประตูเปิดสุดจริงหรือไม่ (สัดส่วนการเปิด >= 98%)
// ใช้เช็กความปลอดภัยฝั่งมอเตอร์ ให้ขับถอยหลังปล่อยหินได้อย่างมั่นใจ
bool doorControl_isOpenFull() {
  return currentOpenFraction >= 0.98f;
}

float doorControl_getOpenFraction() {
  return currentOpenFraction;
}

DoorControlState doorControl_getState() {
  return state;
}

void doorControl_update() {
  unsigned long now = millis();

  switch (state) {

    case DOOR_COLLECT_OPENING:
      // รอเซอร์โวเคลื่อนไปถึงมุมง้างก่อน
      if (now - stateTimer >= MOVE_SETTLE_MS) {
        state = DOOR_COLLECT_HOLDING;
        stateTimer = now;
      }
      break;

    case DOOR_COLLECT_HOLDING:
      // ง้างค้างไว้ตามเวลาที่ตั้ง ให้หินเข้ามาอยู่ในรัศมีปิดประตู
      if (now - stateTimer >= COLLECT_HOLD_MS) {
        setDoorAngles(ANGLE_LEFT_CLOSED, ANGLE_RIGHT_CLOSED);
        currentOpenFraction = 0.0f; // sync ตอนปิดจริงระหว่าง autonomous sequence ด้วย
        state = DOOR_COLLECT_CLOSING;
        stateTimer = now;
      }
      break;

    case DOOR_COLLECT_CLOSING:
      // รอเซอร์โวหุบสุด
      if (now - stateTimer >= MOVE_SETTLE_MS) {
        state = DOOR_IDLE;
      }
      break;

    case DOOR_RELEASE_OPENING:
      if (now - stateTimer >= MOVE_SETTLE_MS) {
        state = DOOR_RELEASE_HOLDING;
        stateTimer = now;
      }
      break;

    case DOOR_RELEASE_HOLDING:
      // เปิดสุดค้างไว้ ให้เช็ก doorControl_isIdle() แล้วสั่งมอเตอร์ถอยรถต่อเอง
      state = DOOR_IDLE;
      break;

    case DOOR_IDLE:
    default:
      break;
  }
}