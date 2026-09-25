/*
วิธีใช้งานคร่าวๆ:
  #include "DoorControl.h"

  void setup() {
    doorControl_begin(18, 19);   // ขาเซอร์โวซ้าย, ขวา
  }

  void loop() {
    doorControl_update();        // ต้องเรียกทุกรอบ loop()

    // Autonomous mode: ระบบตัดสินใจเก็บ/ปล่อยเอง ใช้ลำดับมีเวลาในตัว
    if (autonomous logic เก็บหิน) {
      doorControl_startCollect();
    }
    if (autonomous logic ปล่อยหินทั้งหมด) {
      doorControl_startRelease();
    }
    if (doorControl_isIdle()) {
      // ทำงานเสร็จแล้วจะสั่งอย่างอื่นต่อได้
    }

    // Manual mode (preset): มี 3 state คือ ปิด ง้างเก็บ เปิดเต็ม
    if (gesture ง้างเก็บหิน) doorControl_manualOpenCollect();
    if (gesture ปิดประตู)   doorControl_manualClose();
    if (gesture เปิดสุด)    doorControl_manualOpenFull();

    // Manual mode (proportional): คุมมุมต่อเนื่องด้วยค่า 0.0 - 1.0
    // fraction มาจากภายนอก (0.0 = ปิดสนิท, 1.0 = เปิดสุด) แปลงมุมจริงให้เอง
    doorControl_manualSetOpen(pinchFraction);

    // Fail-safe: เรียกตอนขาดการติดต่อ/timeout
    if (สัญญาณหลุดเกิน timeout) doorControl_failSafe();

    // Telemetry / เช็คสถานะจริง (ไม่ใช่แค่ idle จาก autonomous)
    if (doorControl_isClosed()) {
      // ประตูปิดสนิทจริง ปลอดภัยที่จะสั่งมอเตอร์ถอย/เดินหน้าได้เต็มที่
    }
    if (doorControl_isOpenFull()) {
      // ประตูเปิดสุดจริง ปลอดภัยที่จะสั่งมอเตอร์ "ถอยหลัง" เพื่อทิ้งหิน
    }
  }
*/

#ifndef DOOR_CONTROL_H
#define DOOR_CONTROL_H

#include <Arduino.h>

enum DoorControlState {
  DOOR_IDLE,
  DOOR_COLLECT_OPENING,
  DOOR_COLLECT_HOLDING,
  DOOR_COLLECT_CLOSING,
  DOOR_RELEASE_OPENING,
  DOOR_RELEASE_HOLDING
};

// เรียกครั้งเดียวใน setup() ระบุขา GPIO ที่ต่อเซอร์โวซ้าย/ขวา
void doorControl_begin(int pinLeft, int pinRight);

// เรียกทุกรอบใน loop() เพื่อขยับ state machine
void doorControl_update();

// Autonomous
// เปิดง้างตาม COLLECT_HOLD_MS แล้วดันหินเก็บ
bool doorControl_startCollect();

// ปล่อยหินทั้งหมดแบบอัตโนมัติ
bool doorControl_startRelease();

// Manual (preset)
// ไม่มีรอค้าง เรียกได้ตลอดเวลา จะยกเลิกลำดับอัตโนมัติที่ค้างอยู่ทันที
void doorControl_manualOpenCollect(); // เปิดง้างมุมสำหรับเก็บหิน
void doorControl_manualOpenFull();    // เปิดสุด
void doorControl_manualClose();       // ปิดสนิท

// Manual (proportional)
// คุมมุมประตูต่อเนื่องด้วยค่าเดียว 0.0-1.0 (ใช้กับ gesture/pinch ที่กางนิ้วได้ไม่จำกัดขั้น)
// 0.0 = ปิดสนิท (เทียบเท่า manualClose), 1.0 = เปิดสุด (เทียบเท่า manualOpenFull)
// ค่านอกช่วง 0.0-1.0 จะถูก constrain ให้อัตโนมัติ, ค่า NaN จะถูกเมินทิ้ง (ไม่ขยับ)
// เหมือน manual ตัวอื่นๆ: ไม่มีรอค้าง, ยกเลิกลำดับอัตโนมัติที่ค้างอยู่ทันที
void doorControl_manualSetOpen(float openFraction);

// ปิดสนิททันที (ใช้ตอน reset/เริ่มรอบใหม่) เทียบเท่า manualClose()
void doorControl_goHome();

// เรียกตอนขาดการติดต่อ/หมดเวลา (network timeout) ไม่ว่ากำลังอยู่โหมดไหน/สเตทไหนก็ตาม
// ปิดประตูสนิททันทีเพื่อกันหินหล่นระหว่างค้าง ควรเรียกคู่กับการหยุดมอเตอร์ฝั่ง firmware เสมอ
void doorControl_failSafe();

// true เมื่อประตูว่าง ไม่ได้ทำ autonomous sequence อยู่
// ไม่ได้แปลว่าประตูปิดสนิท หลัง manualOpenFull()/manualSetOpen(1.0) ก็ true เช่นกัน
// ถ้าต้องการเช็คว่าประตูปิดจริงหรือไม่ ให้ใช้ doorControl_isClosed() แทน
bool doorControl_isIdle();

// true เมื่อประตูปิดสนิทจริง (openFraction ~0) ไม่ว่าจะมาจากโหมดไหนก็ตาม
// ใช้แทน isIdle() เวลาต้องการเช็คความปลอดภัยจริงๆ ก่อนสั่งมอเตอร์ต่อ
bool doorControl_isClosed();

// true เมื่อประตูเปิดสุดจริง (openFraction ~1.0) ไม่ว่าจะมาจากโหมดไหนก็ตาม
// ใช้เช็กความปลอดภัยฝั่งมอเตอร์ว่าประตูเปิดสุดพร้อมสั่งถอยหลังทิ้งหินได้อย่างปลอดภัย
bool doorControl_isOpenFull();

// อ่านค่าสัดส่วนการเปิดปัจจุบัน 0.0 (ปิดสนิท) - 1.0 (เปิดสุด) เผื่อ debug/log/ส่ง telemetry กลับ PC
float doorControl_getOpenFraction();

// อ่านสถานะปัจจุบันแบบละเอียด (เผื่อ debug/log)
DoorControlState doorControl_getState();

#endif